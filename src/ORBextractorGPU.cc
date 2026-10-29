/**
 * ORBextractorGPU -- orbslam3_ros2 patch (10). See the header for the design.
 *
 * Everything CUDA-specific is reached through dlopen/dlsym with locally
 * declared prototypes, so this file compiles with no CUDA headers installed.
 * The kernels below are deliberately literal ports of the CPU routines in
 * ORBextractor.cc (OpenCV's FAST-9 corner score, ORB-SLAM3's IC_Angle, the
 * GET_VALUE rBRIEF with cvRound semantics and OpenCV's fastAtan2 polynomial),
 * so that a GPU keypoint set differs from the CPU one only by floating-point
 * and tie-breaking details, not by definition.
 */
#include "ORBextractorGPU.h"

#include <dlfcn.h>
#include <glob.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>

namespace ORB_SLAM3
{

// ---------------------------------------------------------------- driver API
typedef int CUresult; typedef int CUdevice;
typedef void* CUcontext; typedef void* CUmodule; typedef void* CUfunction; typedef void* CUstream;
typedef unsigned long long CUdeviceptr;
typedef void* nvrtcProgram;

struct Api {
    CUresult (*cuInit)(unsigned);
    CUresult (*cuDeviceGet)(CUdevice*, int);
    CUresult (*cuDeviceGetAttribute)(int*, int, CUdevice);
    CUresult (*cuDevicePrimaryCtxRetain)(CUcontext*, CUdevice);
    CUresult (*cuCtxSetCurrent)(CUcontext);
    CUresult (*cuModuleLoadData)(CUmodule*, const void*);
    CUresult (*cuModuleGetFunction)(CUfunction*, CUmodule, const char*);
    CUresult (*cuMemAlloc)(CUdeviceptr*, size_t);
    CUresult (*cuMemFree)(CUdeviceptr);
    CUresult (*cuMemcpyHtoDAsync)(CUdeviceptr, const void*, size_t, CUstream);
    CUresult (*cuMemcpyDtoHAsync)(void*, CUdeviceptr, size_t, CUstream);
    CUresult (*cuMemcpyHtoD)(CUdeviceptr, const void*, size_t);
    CUresult (*cuMemsetD8Async)(CUdeviceptr, unsigned char, size_t, CUstream);
    CUresult (*cuMemsetD32Async)(CUdeviceptr, unsigned, size_t, CUstream);
    CUresult (*cuStreamCreate)(CUstream*, unsigned);
    CUresult (*cuStreamSynchronize)(CUstream);
    CUresult (*cuLaunchKernel)(CUfunction, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, CUstream, void**, void**);
    CUresult (*cuGetErrorString)(CUresult, const char**);
    int (*nvrtcCreateProgram)(nvrtcProgram*, const char*, const char*, int, const char* const*, const char* const*);
    int (*nvrtcCompileProgram)(nvrtcProgram, int, const char* const*);
    int (*nvrtcGetPTXSize)(nvrtcProgram, size_t*);
    int (*nvrtcGetPTX)(nvrtcProgram, char*);
    int (*nvrtcGetProgramLogSize)(nvrtcProgram, size_t*);
    int (*nvrtcGetProgramLog)(nvrtcProgram, char*);
    int (*nvrtcDestroyProgram)(nvrtcProgram*);
};

static Api g_api; static CUcontext g_ctx = nullptr; static CUmodule g_mod = nullptr;
static CUfunction g_kFast = nullptr, g_kCellMax = nullptr, g_kCompact = nullptr, g_kBlur = nullptr, g_kOrient = nullptr, g_kBrief = nullptr;
static CUfunction g_kKnn2 = nullptr;   // patch (10b)
static bool g_tried = false, g_ok = false; static std::string g_reason; static std::mutex g_mutex;

// ------------------------------------------------------------------ kernels
static const char* kKernelSource = R"CUDA(
extern "C" {

// Brute-force Hamming 2-NN (orbslam3_ros2 patch (10b), 2026-10-29). One thread
// per query descriptor (256 bits = 4 x u64 held in registers); the train set
// streams through shared memory in tiles of 256 descriptors. Strict "<"
// updates reproduce cv::batchDistance's tie order: the lower train index wins.
// out[q*4 + 0..3] = best index, best distance, second index, second distance.
__global__ void knn2(const unsigned long long* q, int nq, const unsigned long long* t, int nt, int* out)
{
    __shared__ unsigned long long st[256 * 4];
    int qi = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned long long q0 = 0, q1 = 0, q2 = 0, q3 = 0;
    if (qi < nq) { q0 = q[qi * 4]; q1 = q[qi * 4 + 1]; q2 = q[qi * 4 + 2]; q3 = q[qi * 4 + 3]; }
    int b0 = 1 << 30, i0 = -1, b1 = 1 << 30, i1 = -1;
    for (int base = 0; base < nt; base += 256) {
        int n = nt - base; if (n > 256) n = 256;
        for (int k = threadIdx.x; k < n * 4; k += blockDim.x) st[k] = t[(size_t)base * 4 + k];
        __syncthreads();
        if (qi < nq) {
            for (int j = 0; j < n; ++j) {
                int d = __popcll(q0 ^ st[j * 4]) + __popcll(q1 ^ st[j * 4 + 1]) + __popcll(q2 ^ st[j * 4 + 2]) + __popcll(q3 ^ st[j * 4 + 3]);
                if (d < b0) { b1 = b0; i1 = i0; b0 = d; i0 = base + j; }
                else if (d < b1) { b1 = d; i1 = base + j; }
            }
        }
        __syncthreads();
    }
    if (qi < nq) { out[qi * 4] = i0; out[qi * 4 + 1] = b0; out[qi * 4 + 2] = i1; out[qi * 4 + 3] = b1; }
}

// OpenCV FAST-9 (patternSize 16, K = 8) corner score: the largest threshold
// for which the segment test still passes; 0 when not a corner. Literal port
// of cv::cornerScore<16>. `score` is written for pixels inside the detection
// rectangle [x0,x1) x [y0,y1) of the level ROI; 0 elsewhere.
__global__ void fast_score(const unsigned char* img, int step, int ox, int oy, int w, int h,
                           int x0, int y0, int x1, int y1, unsigned char* score)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) return;
    int idx = y * w + x;
    if (x < x0 || x >= x1 || y < y0 || y >= y1) { score[idx] = 0; return; }
    const unsigned char* p = img + (oy + y) * step + ox + x;
    const int dx[16] = {0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
    const int dy[16] = {-3,-3,-2,-1,0,1,2,3,3,3,2,1,0,-1,-2,-3};
    int v = p[0]; int d[25];
    for (int k = 0; k < 16; ++k) d[k] = v - (int)p[dy[k] * step + dx[k]];
    for (int k = 16; k < 25; ++k) d[k] = d[k - 16];
    int a0 = 0;
    for (int k = 0; k < 16; k += 2) {
        int a = min(d[k+1], d[k+2]); a = min(a, d[k+3]);
        if (a <= a0) continue;
        a = min(a, d[k+4]); a = min(a, d[k+5]); a = min(a, d[k+6]); a = min(a, d[k+7]); a = min(a, d[k+8]);
        a0 = max(a0, min(a, d[k])); a0 = max(a0, min(a, d[k+9]));
    }
    int b0 = -a0;
    for (int k = 0; k < 16; k += 2) {
        int b = max(d[k+1], d[k+2]); b = max(b, d[k+3]);
        if (b >= b0) continue;
        b = max(b, d[k+4]); b = max(b, d[k+5]); b = max(b, d[k+6]); b = max(b, d[k+7]); b = max(b, d[k+8]);
        b0 = min(b0, max(b, d[k])); b0 = min(b0, max(b, d[k+9]));
    }
    int s = -b0 - 1;
    score[idx] = (unsigned char)(s < 0 ? 0 : (s > 255 ? 255 : s));
}

// Per-cell maximum score (ORB-SLAM3's cell grid), for the two-threshold rule.
__global__ void cell_max(const unsigned char* score, int w, int h, int minBx, int minBy,
                         int wCell, int hCell, int nCols, int nRows, int* cmax)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) return;
    int s = score[y * w + x]; if (!s) return;
    int j = (x - minBx) / wCell, i = (y - minBy) / hCell;
    if (j < 0 || i < 0 || j >= nCols || i >= nRows) return;
    atomicMax(&cmax[i * nCols + j], s);
}

// 3x3 non-max suppression + cell rule (keep the iniTh set if the cell has one,
// else the minTh set) + compaction to (x-minBx, y-minBy, score) triples.
__global__ void compact(const unsigned char* score, int w, int h, int minBx, int minBy,
                        int wCell, int hCell, int nCols, int nRows, int iniTh, int minTh,
                        const int* cmax, int* out, int cap, int* count)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x < 1 || y < 1 || x >= w - 1 || y >= h - 1) return;
    int s = score[y * w + x]; if (s < minTh) return;
    int j = (x - minBx) / wCell, i = (y - minBy) / hCell;
    if (j < 0 || i < 0 || j >= nCols || i >= nRows) return;
    int th = (cmax[i * nCols + j] >= iniTh) ? iniTh : minTh;
    if (s < th) return;
    const unsigned char* r0 = score + (y - 1) * w + x, *r1 = score + y * w + x, *r2 = score + (y + 1) * w + x;
    if (s <= r0[-1] || s <= r0[0] || s <= r0[1] || s <= r1[-1] || s < r1[1] || s < r2[-1] || s < r2[0] || s < r2[1]) return;
    int k = atomicAdd(count, 1); if (k >= cap) return;
    out[3 * k] = x - minBx; out[3 * k + 1] = y - minBy; out[3 * k + 2] = s;
}

// 7x7 Gaussian, sigma 2, BORDER_REFLECT_101 inside the level ROI; writes the
// blurred ROI (no border) at `dst` with pitch w.
__global__ void blur7(const unsigned char* img, int step, int ox, int oy, int w, int h, unsigned char* dst)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) return;
    const float g[7] = {0.07015933f, 0.13107488f, 0.19071282f, 0.21610594f, 0.19071282f, 0.13107488f, 0.07015933f};
    float acc = 0.f;
    for (int ky = -3; ky <= 3; ++ky) {
        int yy = y + ky; if (yy < 0) yy = -yy; if (yy >= h) yy = 2 * h - 2 - yy;
        const unsigned char* row = img + (oy + yy) * step + ox;
        float racc = 0.f;
        for (int kx = -3; kx <= 3; ++kx) {
            int xx = x + kx; if (xx < 0) xx = -xx; if (xx >= w) xx = 2 * w - 2 - xx;
            racc += g[kx + 3] * row[xx];
        }
        acc += g[ky + 3] * racc;
    }
    int v = __float2int_rn(acc); dst[y * w + x] = (unsigned char)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

// OpenCV's fastAtan2 (degrees, polynomial), so angles match the CPU path.
__device__ float fast_atan2(float y, float x)
{
    const float p1 = 57.28590f, p3 = -18.66728f, p5 = 8.913745f, p7 = -2.539748f;
    float ax = fabsf(x), ay = fabsf(y), a, c, c2;
    if (ax >= ay) { c = ay / (ax + 1.1920929e-7f); c2 = c * c; a = (((p7 * c2 + p5) * c2 + p3) * c2 + p1) * c; }
    else          { c = ax / (ay + 1.1920929e-7f); c2 = c * c; a = 90.f - (((p7 * c2 + p5) * c2 + p3) * c2 + p1) * c; }
    if (x < 0) a = 180.f - a;
    if (y < 0) a = 360.f - a;
    return a;
}

// ORB-SLAM3 IC_Angle on the UNBLURRED level image; kps = (x, y) in level coords.
__global__ void orientation(const unsigned char* img, int step, int ox, int oy, const float* kps, int n,
                            const int* umax, float* angle)
{
    int k = blockIdx.x * blockDim.x + threadIdx.x; if (k >= n) return;
    int cx = __float2int_rn(kps[2 * k]), cy = __float2int_rn(kps[2 * k + 1]);
    const unsigned char* c = img + (oy + cy) * step + ox + cx;
    int m01 = 0, m10 = 0;
    for (int u = -15; u <= 15; ++u) m10 += u * c[u];
    for (int v = 1; v <= 15; ++v) {
        int vsum = 0, d = umax[v];
        for (int u = -d; u <= d; ++u) { int vp = c[u + v * step], vm = c[u - v * step]; vsum += vp - vm; m10 += u * (vp + vm); }
        m01 += v * vsum;
    }
    angle[k] = fast_atan2((float)m01, (float)m10);
}

// Rotated rBRIEF on the BLURRED level ROI (pitch w): literal GET_VALUE port.
__global__ void brief(const unsigned char* blurred, int w, const float* kps, const float* angle, int n,
                      const int* pattern, unsigned char* desc)
{
    int k = blockIdx.x * blockDim.x + threadIdx.x; if (k >= n) return;
    float ang = angle[k] * 0.017453292519943295f; float a = cosf(ang), b = sinf(ang);
    int cx = __float2int_rn(kps[2 * k]), cy = __float2int_rn(kps[2 * k + 1]);
    const unsigned char* c = blurred + cy * w + cx;
    const int* pat = pattern;
    for (int i = 0; i < 32; ++i, pat += 32) {
        int val = 0;
        for (int j = 0; j < 8; ++j) {
            int px0 = pat[4 * j], py0 = pat[4 * j + 1], px1 = pat[4 * j + 2], py1 = pat[4 * j + 3];
            int t0 = c[__float2int_rn(px0 * b + py0 * a) * w + __float2int_rn(px0 * a - py0 * b)];
            int t1 = c[__float2int_rn(px1 * b + py1 * a) * w + __float2int_rn(px1 * a - py1 * b)];
            val |= (t0 < t1) << j;
        }
        desc[32 * k + i] = (unsigned char)val;
    }
}
}
)CUDA";

// --------------------------------------------------------------- loading
static void* dlopenFirst(const std::vector<std::string>& cands) {
    for (const auto& c : cands) { void* h = dlopen(c.c_str(), RTLD_NOW | RTLD_GLOBAL); if (h) return h; }
    return nullptr;
}
static std::vector<std::string> nvrtcCandidates() {
    std::vector<std::string> v;
    if (const char* d = getenv("ORB_SLAM3_NVRTC_DIR")) v.push_back(std::string(d) + "/libnvrtc.so.12");
    v.push_back("libnvrtc.so.12");
    if (const char* home = getenv("HOME")) {
        glob_t g; std::string pat = std::string(home) + "/.local/lib/python3*/site-packages/nvidia/cuda_nvrtc/lib/libnvrtc.so.12";
        if (glob(pat.c_str(), 0, nullptr, &g) == 0) { for (size_t i = 0; i < g.gl_pathc; ++i) v.push_back(g.gl_pathv[i]); globfree(&g); }
    }
    v.push_back("/usr/local/cuda/lib64/libnvrtc.so.12");
    return v;
}
#define LOAD(h, name) do { g_api.name = (decltype(g_api.name))dlsym(h, #name); if (!g_api.name) { g_reason = std::string("missing symbol ") + #name; return false; } } while (0)
#define LOADV(h, name, sym) do { g_api.name = (decltype(g_api.name))dlsym(h, sym); if (!g_api.name) { g_reason = std::string("missing symbol ") + sym; return false; } } while (0)

static bool initProcess() {
    void* hcu = dlopen("libcuda.so.1", RTLD_NOW | RTLD_GLOBAL);
    if (!hcu) { g_reason = "libcuda.so.1 not loadable (no NVIDIA driver?)"; return false; }
    void* hrt = dlopenFirst(nvrtcCandidates());
    if (!hrt) { g_reason = "libnvrtc.so.12 not found (set ORB_SLAM3_NVRTC_DIR)"; return false; }
    LOAD(hcu, cuInit); LOAD(hcu, cuDeviceGet); LOAD(hcu, cuDeviceGetAttribute); LOAD(hcu, cuDevicePrimaryCtxRetain);
    LOAD(hcu, cuCtxSetCurrent); LOAD(hcu, cuModuleLoadData); LOAD(hcu, cuModuleGetFunction); LOAD(hcu, cuGetErrorString);
    LOADV(hcu, cuMemAlloc, "cuMemAlloc_v2"); LOADV(hcu, cuMemFree, "cuMemFree_v2");
    LOADV(hcu, cuMemcpyHtoDAsync, "cuMemcpyHtoDAsync_v2"); LOADV(hcu, cuMemcpyDtoHAsync, "cuMemcpyDtoHAsync_v2");
    LOADV(hcu, cuMemcpyHtoD, "cuMemcpyHtoD_v2"); LOAD(hcu, cuMemsetD8Async); LOAD(hcu, cuMemsetD32Async);
    LOAD(hcu, cuStreamCreate); LOAD(hcu, cuStreamSynchronize); LOAD(hcu, cuLaunchKernel);
    LOAD(hrt, nvrtcCreateProgram); LOAD(hrt, nvrtcCompileProgram); LOAD(hrt, nvrtcGetPTXSize); LOAD(hrt, nvrtcGetPTX);
    LOAD(hrt, nvrtcGetProgramLogSize); LOAD(hrt, nvrtcGetProgramLog); LOAD(hrt, nvrtcDestroyProgram);
    if (g_api.cuInit(0) != 0) { g_reason = "cuInit failed"; return false; }
    CUdevice dev; if (g_api.cuDeviceGet(&dev, 0) != 0) { g_reason = "no CUDA device"; return false; }
    int major = 0, minor = 0; g_api.cuDeviceGetAttribute(&major, 75, dev); g_api.cuDeviceGetAttribute(&minor, 76, dev);
    if (g_api.cuDevicePrimaryCtxRetain(&g_ctx, dev) != 0 || g_api.cuCtxSetCurrent(g_ctx) != 0) { g_reason = "primary context"; return false; }
    nvrtcProgram prog; if (g_api.nvrtcCreateProgram(&prog, kKernelSource, "orb.cu", 0, nullptr, nullptr) != 0) { g_reason = "nvrtcCreateProgram"; return false; }
    char arch[64]; snprintf(arch, sizeof arch, "--gpu-architecture=compute_%d%d", major, minor);
    const char* opts[] = { arch, "--use_fast_math" };
    if (g_api.nvrtcCompileProgram(prog, 2, opts) != 0) {
        size_t n = 0; g_api.nvrtcGetProgramLogSize(prog, &n); std::string log(n, '\0'); g_api.nvrtcGetProgramLog(prog, &log[0]);
        g_reason = "NVRTC compile failed: " + log.substr(0, 600); g_api.nvrtcDestroyProgram(&prog); return false;
    }
    size_t n = 0; g_api.nvrtcGetPTXSize(prog, &n); std::string ptx(n, '\0'); g_api.nvrtcGetPTX(prog, &ptx[0]); g_api.nvrtcDestroyProgram(&prog);
    if (g_api.cuModuleLoadData(&g_mod, ptx.c_str()) != 0) { g_reason = "cuModuleLoadData"; return false; }
    struct { const char* n; CUfunction* f; } fns[] = { {"fast_score", &g_kFast}, {"cell_max", &g_kCellMax}, {"compact", &g_kCompact},
                                                       {"blur7", &g_kBlur}, {"orientation", &g_kOrient}, {"brief", &g_kBrief},
                                                       {"knn2", &g_kKnn2} };
    for (auto& e : fns) if (g_api.cuModuleGetFunction(e.f, g_mod, e.n) != 0) { g_reason = std::string("kernel ") + e.n; return false; }
    return true;
}

bool GpuOrb::Available() {
    std::lock_guard<std::mutex> lk(g_mutex);
    if (!g_tried) { g_tried = true; g_ok = initProcess(); if (!g_ok) std::cerr << "[ORBextractorGPU] GPU path unavailable: " << g_reason << " -- using the CPU extractor" << std::endl; }
    return g_ok;
}
const std::string& GpuOrb::Reason() { return g_reason; }

// ------------------------------------------------------------ per instance
struct GpuOrb::Impl {
    CUstream stream = nullptr; int nlevels = 0, iniTh = 20, minTh = 7; size_t imgBytes = 0, roiBytes = 0;
    std::vector<CUdeviceptr> dImg, dBlur; std::vector<int> step, ox, oy, w, h;
    CUdeviceptr dScore = 0, dCellMax = 0, dCorners = 0, dCount = 0, dKps = 0, dAngle = 0, dDesc = 0, dUmax = 0, dPattern = 0;
    int cap = 0, kpCap = 0; std::vector<int> hostCorners;
    bool launch(CUfunction f, unsigned gx, unsigned gy, unsigned bx, unsigned by, void** args) {
        return g_api.cuLaunchKernel(f, gx, gy, 1, bx, by, 1, 0, stream, args, nullptr) == 0;
    }
    ~Impl() { for (auto p : dImg) if (p) g_api.cuMemFree(p); for (auto p : dBlur) if (p) g_api.cuMemFree(p);
              for (CUdeviceptr p : {dScore, dCellMax, dCorners, dCount, dKps, dAngle, dDesc, dUmax, dPattern}) if (p) g_api.cuMemFree(p); }
};

GpuOrb::GpuOrb() : p_(new Impl) {}
GpuOrb::~GpuOrb() { delete p_; }

bool GpuOrb::Init(int nlevels, int maxWholeW, int maxWholeH, int maxStep, int iniThFAST, int minThFAST,
                  const std::vector<int>& umax, const std::vector<cv::Point>& pattern) {
    if (!Available()) return false;
    g_api.cuCtxSetCurrent(g_ctx);
    Impl& s = *p_; s.nlevels = nlevels; s.iniTh = iniThFAST; s.minTh = minThFAST;
    s.imgBytes = (size_t)maxStep * maxWholeH; s.roiBytes = (size_t)maxWholeW * maxWholeH;
    s.cap = 120000; s.kpCap = 20000;
    if (g_api.cuStreamCreate(&s.stream, 1) != 0) return false;
    s.dImg.assign(nlevels, 0); s.dBlur.assign(nlevels, 0); s.step.assign(nlevels, 0); s.ox.assign(nlevels, 0); s.oy.assign(nlevels, 0); s.w.assign(nlevels, 0); s.h.assign(nlevels, 0);
    for (int l = 0; l < nlevels; ++l) if (g_api.cuMemAlloc(&s.dImg[l], s.imgBytes) != 0 || g_api.cuMemAlloc(&s.dBlur[l], s.roiBytes) != 0) return false;
    if (g_api.cuMemAlloc(&s.dScore, s.roiBytes) != 0 || g_api.cuMemAlloc(&s.dCellMax, 4096 * sizeof(int)) != 0 ||
        g_api.cuMemAlloc(&s.dCorners, (size_t)s.cap * 3 * sizeof(int)) != 0 || g_api.cuMemAlloc(&s.dCount, sizeof(int)) != 0 ||
        g_api.cuMemAlloc(&s.dKps, (size_t)s.kpCap * 2 * sizeof(float)) != 0 || g_api.cuMemAlloc(&s.dAngle, (size_t)s.kpCap * sizeof(float)) != 0 ||
        g_api.cuMemAlloc(&s.dDesc, (size_t)s.kpCap * 32) != 0 || g_api.cuMemAlloc(&s.dUmax, 16 * sizeof(int)) != 0 ||
        g_api.cuMemAlloc(&s.dPattern, 1024 * sizeof(int)) != 0) return false;
    std::vector<int> um(16, 0); for (size_t i = 0; i < umax.size() && i < 16; ++i) um[i] = umax[i];
    std::vector<int> pat(1024, 0); for (size_t i = 0; i < pattern.size() && i < 512; ++i) { pat[2 * i] = pattern[i].x; pat[2 * i + 1] = pattern[i].y; }
    if (g_api.cuMemcpyHtoD(s.dUmax, um.data(), 16 * sizeof(int)) != 0 || g_api.cuMemcpyHtoD(s.dPattern, pat.data(), 1024 * sizeof(int)) != 0) return false;
    s.hostCorners.resize((size_t)s.cap * 3);
    return true;
}

bool GpuOrb::Upload(int level, const cv::Mat& m) {
    Impl& s = *p_; if (level < 0 || level >= s.nlevels || m.type() != CV_8UC1) return false;
    g_api.cuCtxSetCurrent(g_ctx);
    cv::Size whole; cv::Point ofs; m.locateROI(whole, ofs);
    const uchar* base = m.data - ofs.y * m.step - ofs.x;
    size_t bytes = (size_t)m.step * whole.height; if (bytes > s.imgBytes) return false;
    s.step[level] = (int)m.step; s.ox[level] = ofs.x; s.oy[level] = ofs.y; s.w[level] = m.cols; s.h[level] = m.rows;
    return g_api.cuMemcpyHtoDAsync(s.dImg[level], base, bytes, s.stream) == 0;
}

bool GpuOrb::Detect(int level, int minBx, int minBy, int maxBx, int maxBy, int wCell, int hCell, int nCols, int nRows, std::vector<cv::KeyPoint>& out) {
    Impl& s = *p_; out.clear(); g_api.cuCtxSetCurrent(g_ctx);
    int w = s.w[level], h = s.h[level]; if (!w || !h || nCols * nRows > 4096) return false;
    // OpenCV's FAST never reports a corner within 3 px of the cell ROI it is
    // given; the cells cover [minB, maxB) so the detectable rectangle is that
    // range shrunk by 3 on every side.
    int x0 = minBx + 3, y0 = minBy + 3, x1 = maxBx - 3, y1 = maxBy - 3;
    unsigned char* dummy = nullptr; (void)dummy;
    void* a1[] = { &s.dImg[level], &s.step[level], &s.ox[level], &s.oy[level], &w, &h, &x0, &y0, &x1, &y1, &s.dScore };
    unsigned gx = (w + 31) / 32, gy = (h + 7) / 8;
    if (!s.launch(g_kFast, gx, gy, 32, 8, a1)) return false;
    if (g_api.cuMemsetD32Async(s.dCellMax, 0, nCols * nRows, s.stream) != 0 || g_api.cuMemsetD32Async(s.dCount, 0, 1, s.stream) != 0) return false;
    void* a2[] = { &s.dScore, &w, &h, &minBx, &minBy, &wCell, &hCell, &nCols, &nRows, &s.dCellMax };
    if (!s.launch(g_kCellMax, gx, gy, 32, 8, a2)) return false;
    void* a3[] = { &s.dScore, &w, &h, &minBx, &minBy, &wCell, &hCell, &nCols, &nRows, &s.iniTh, &s.minTh, &s.dCellMax, &s.dCorners, &s.cap, &s.dCount };
    if (!s.launch(g_kCompact, gx, gy, 32, 8, a3)) return false;
    int count = 0;
    if (g_api.cuMemcpyDtoHAsync(&count, s.dCount, sizeof(int), s.stream) != 0 || g_api.cuStreamSynchronize(s.stream) != 0) return false;
    if (count > s.cap) count = s.cap;
    if (count == 0) return true;
    if (g_api.cuMemcpyDtoHAsync(s.hostCorners.data(), s.dCorners, (size_t)count * 3 * sizeof(int), s.stream) != 0 || g_api.cuStreamSynchronize(s.stream) != 0) return false;
    out.reserve(count);
    for (int k = 0; k < count; ++k) out.emplace_back((float)s.hostCorners[3 * k], (float)s.hostCorners[3 * k + 1], 7.f, -1.f, (float)s.hostCorners[3 * k + 2]);
    return true;
}

static bool uploadKps(GpuOrb::Impl& s, const std::vector<cv::KeyPoint>& kps) {
    std::vector<float> xy(kps.size() * 2);
    for (size_t i = 0; i < kps.size(); ++i) { xy[2 * i] = kps[i].pt.x; xy[2 * i + 1] = kps[i].pt.y; }
    return g_api.cuMemcpyHtoDAsync(s.dKps, xy.data(), xy.size() * sizeof(float), s.stream) == 0;
}

bool GpuOrb::Orientation(int level, std::vector<cv::KeyPoint>& kps) {
    Impl& s = *p_; int n = (int)kps.size(); if (n == 0) return true; if (n > s.kpCap) return false;
    g_api.cuCtxSetCurrent(g_ctx);
    if (!uploadKps(s, kps)) return false;
    void* a[] = { &s.dImg[level], &s.step[level], &s.ox[level], &s.oy[level], &s.dKps, &n, &s.dUmax, &s.dAngle };
    if (!s.launch(g_kOrient, (n + 127) / 128, 1, 128, 1, a)) return false;
    std::vector<float> ang(n);
    if (g_api.cuMemcpyDtoHAsync(ang.data(), s.dAngle, n * sizeof(float), s.stream) != 0 || g_api.cuStreamSynchronize(s.stream) != 0) return false;
    for (int i = 0; i < n; ++i) kps[i].angle = ang[i];
    return true;
}

bool GpuOrb::Describe(int level, const std::vector<cv::KeyPoint>& kps, cv::Mat& desc) {
    Impl& s = *p_; int n = (int)kps.size(); if (n == 0) return true; if (n > s.kpCap || desc.rows != n || desc.cols != 32) return false;
    g_api.cuCtxSetCurrent(g_ctx);
    int w = s.w[level], h = s.h[level];
    void* ab[] = { &s.dImg[level], &s.step[level], &s.ox[level], &s.oy[level], &w, &h, &s.dBlur[level] };
    if (!s.launch(g_kBlur, (w + 31) / 32, (h + 7) / 8, 32, 8, ab)) return false;
    if (!uploadKps(s, kps)) return false;
    std::vector<float> ang(n); for (int i = 0; i < n; ++i) ang[i] = kps[i].angle;
    if (g_api.cuMemcpyHtoDAsync(s.dAngle, ang.data(), n * sizeof(float), s.stream) != 0) return false;
    void* a[] = { &s.dBlur[level], &w, &s.dKps, &s.dAngle, &n, &s.dPattern, &s.dDesc };
    if (!s.launch(g_kBrief, (n + 127) / 128, 1, 128, 1, a)) return false;
    if (!desc.isContinuous()) return false;
    if (g_api.cuMemcpyDtoHAsync(desc.data, s.dDesc, (size_t)n * 32, s.stream) != 0 || g_api.cuStreamSynchronize(s.stream) != 0) return false;
    return true;
}

// ------------------------------------------------- GpuHamming (patch 10b)
namespace {
struct HammingState {
    CUstream stream = nullptr; CUdeviceptr dQ = 0, dT = 0, dOut = 0; int cap = 0;
    std::vector<int> hostOut; bool ready = false, failed = false;
};
HammingState g_ham; std::mutex g_hamMutex;
}

bool GpuHamming::Knn2(const cv::Mat& query, const cv::Mat& train, std::vector<std::vector<cv::DMatch> >& matches) {
    if (query.type() != CV_8UC1 || train.type() != CV_8UC1 || query.cols != 32 || train.cols != 32) return false;
    int nq = query.rows, nt = train.rows;
    if (nq == 0 || nt < 2) return false;            // degenerate: leave it to the CPU matcher
    if (!GpuOrb::Available()) return false;
    std::lock_guard<std::mutex> lk(g_hamMutex);
    HammingState& s = g_ham; if (s.failed) return false;
    g_api.cuCtxSetCurrent(g_ctx);
    if (!s.ready) {
        s.cap = 20000;
        if (g_api.cuStreamCreate(&s.stream, 1) != 0 || g_api.cuMemAlloc(&s.dQ, (size_t)s.cap * 32) != 0 ||
            g_api.cuMemAlloc(&s.dT, (size_t)s.cap * 32) != 0 || g_api.cuMemAlloc(&s.dOut, (size_t)s.cap * 4 * sizeof(int)) != 0) {
            s.failed = true; std::cerr << "[Frame] GPU stereo matcher unavailable (buffer setup failed) -- using cv::BFMatcher" << std::endl; return false;
        }
        s.hostOut.resize((size_t)s.cap * 4); s.ready = true;
        std::cout << "[Frame] GPU stereo matcher active (brute-force Hamming 2-NN on the GPU)" << std::endl;
    }
    if (nq > s.cap || nt > s.cap) return false;
    cv::Mat q = query.isContinuous() ? query : query.clone(), t = train.isContinuous() ? train : train.clone();
    if (g_api.cuMemcpyHtoDAsync(s.dQ, q.data, (size_t)nq * 32, s.stream) != 0 ||
        g_api.cuMemcpyHtoDAsync(s.dT, t.data, (size_t)nt * 32, s.stream) != 0) return false;
    void* args[] = { &s.dQ, &nq, &s.dT, &nt, &s.dOut };
    if (g_api.cuLaunchKernel(g_kKnn2, (nq + 63) / 64, 1, 1, 64, 1, 1, 0, s.stream, args, nullptr) != 0) return false;
    if (g_api.cuMemcpyDtoHAsync(s.hostOut.data(), s.dOut, (size_t)nq * 4 * sizeof(int), s.stream) != 0 ||
        g_api.cuStreamSynchronize(s.stream) != 0) return false;
    matches.clear(); matches.resize(nq);
    for (int i = 0; i < nq; ++i) {
        const int* o = &s.hostOut[(size_t)i * 4];
        matches[i].reserve(2);
        matches[i].push_back(cv::DMatch(i, o[0], 0, (float)o[1]));
        matches[i].push_back(cv::DMatch(i, o[2], 0, (float)o[3]));
    }
    return true;
}

} // namespace ORB_SLAM3

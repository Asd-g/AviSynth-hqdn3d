/*
    Copyright (C) 2003 Daniel Moreno <comac@comac.darktech.org>
    Avisynth port (C) 2005 Loren Merritt <lorenm@u.washington.edu>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <algorithm>
#include <cmath>

#include "avisynth.h"

typedef union {
    uint16_t u16;
    uint8_t  u8[2];
} av_alias;

#define AV_RN16A(p) (((const av_alias*)(p))->u16)
#define AV_WN16A(p, v) (((av_alias*)(p))->u16 = (v))

#define LUT_BITS (depth==16 ? 8 : 4)
#define LOAD(x) (((depth == 8 ? src[x] : AV_RN16A(src + (x) * 2)) << (16 - depth)) + (((1 << (16 - depth)) - 1) >> 1))
#define STORE(x,val) (depth == 8 ? dst[x] = (val) >> (16 - depth) : AV_WN16A(dst + (x) * 2, (val) >> (16 - depth)))

class hqdn3d : public GenericVideoFilter
{
    int16_t* coefs[4];
    uint16_t* line[3];
    uint16_t* frame_prev[3];
    int planecount;
    int process[3];
    bool has_at_least_v8;
    int _restartlap;
    int prev_frame = -99999;
    PVideoFrame cache;

    PVideoFrame filterFrame(PVideoFrame& src, IScriptEnvironment* env);

public:
    hqdn3d(PClip _child, double LumSpac, double ChromSpac, double LumTmp, double ChromTmp, int restart, int y, int u, int v, IScriptEnvironment* env);
    ~hqdn3d();
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
};

static inline uint32_t lowpass(int prev, int cur, int16_t* coef, int depth)
{
    const int d = (prev - cur) >> (8 - LUT_BITS);
    return cur + coef[d];
}

static inline void denoise_temporal(const uint8_t* src, uint8_t* dst, uint16_t* frame_ant, int w, int h, int sstride, int dstride, int16_t* temporal, int depth)
{
    long x, y;
    uint32_t tmp;

    temporal += 256 << LUT_BITS;

    for (y = 0; y < h; ++y)
    {
        for (x = 0; x < w; ++x)
        {
            frame_ant[x] = tmp = lowpass(frame_ant[x], LOAD(x), temporal, depth);
            STORE(x, tmp);
        }

        src += sstride;
        dst += dstride;
        frame_ant += w;
    }
}

static inline void denoise_spatial(const uint8_t* src, uint8_t* dst, uint16_t* line_ant, uint16_t* frame_ant, int w, int h, int sstride, int dstride, int16_t* spatial, int16_t* temporal, int depth)
{
    long x, y;
    uint32_t pixel_ant;
    uint32_t tmp;

    spatial += 256 << LUT_BITS;
    temporal += 256 << LUT_BITS;

    /* First line has no top neighbor. Only left one for each tmp and
     * last frame */
    pixel_ant = LOAD(0);
    for (x = 0; x < w; ++x)
    {
        line_ant[x] = tmp = pixel_ant = lowpass(pixel_ant, LOAD(x), spatial, depth);
        frame_ant[x] = tmp = lowpass(frame_ant[x], tmp, temporal, depth);
        STORE(x, tmp);
    }

    w /= (depth == 8) ? 1 : 2;

    for (y = 1; y < h; ++y)
    {
        src += sstride;
        dst += dstride;
        frame_ant += w;

        pixel_ant = LOAD(0);
        for (x = 0; x < w - 1; ++x)
        {
            line_ant[x] = tmp = lowpass(line_ant[x], pixel_ant, spatial, depth);
            pixel_ant = lowpass(pixel_ant, LOAD(x + 1), spatial, depth);
            frame_ant[x] = tmp = lowpass(frame_ant[x], tmp, temporal, depth);
            STORE(x, tmp);
        }
        line_ant[x] = tmp = lowpass(line_ant[x], pixel_ant, spatial, depth);
        frame_ant[x] = tmp = lowpass(frame_ant[x], tmp, temporal, depth);
        STORE(x, tmp);
    }
}

static inline void denoise(const uint8_t* src, uint8_t* dst, uint16_t* line_ant, uint16_t** frame_ant_ptr, int w, int h, int sstride, int dstride, int16_t* spatial, int16_t* temporal, int depth)
{
    if (spatial[0])
        denoise_spatial(src, dst, line_ant, *frame_ant_ptr, w, h, sstride, dstride, spatial, temporal, depth);
    else
        denoise_temporal(src, dst, *frame_ant_ptr, w, h, sstride, dstride, temporal, depth);
}

PVideoFrame hqdn3d::filterFrame(PVideoFrame& src, IScriptEnvironment* env)
{
    PVideoFrame dst = (has_at_least_v8) ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi);
    const int planes[3] = { PLANAR_Y, PLANAR_U, PLANAR_V };

    for (int i = 0; i < planecount; ++i)
    {
        const int pitch = src->GetPitch(planes[i]);
        const int dst_pitch = dst->GetPitch(planes[i]);
        const int width = src->GetRowSize(planes[i]);
        const int height = src->GetHeight(planes[i]);
        const uint8_t* srcp = src->GetReadPtr(planes[i]);
        uint8_t* dstp = dst->GetWritePtr(planes[i]);

        if (process[i] == 3)
            denoise(srcp, dstp, line[i], &frame_prev[i], width, height, pitch, dst_pitch, coefs[(i) ? 2 : 0], coefs[(i) ? 3 : 1], vi.BitsPerComponent());
        else if (process[i] == 2)
            env->BitBlt(dstp, dst_pitch, srcp, pitch, width, height);
    }

    return dst;
}

static void precalc_coefs(double dist25, int depth, int16_t* ct)
{
    int i;
    double gamma, simil, C;

    gamma = log(0.25) / log(1.0 - std::min(dist25, 252.0) / 255.0 - 0.00001);

    for (i = -256 << LUT_BITS; i < 256 << LUT_BITS; ++i)
    {
        double f = ((i << (9 - LUT_BITS)) + (1 << (8 - LUT_BITS)) - 1) / 512.0; // midpoint of the bin
        simil = std::max(0.0, 1.0 - fabs(f) / 255.0);
        C = pow(simil, gamma) * 256.0 * f;
        ct[(256 << LUT_BITS) + i] = lrint(C);
    }

    ct[0] = !!dist25;
}

hqdn3d::hqdn3d(PClip _child, double LumSpac, double ChromSpac, double LumTmp, double ChromTmp, int restart, int y, int u, int v, IScriptEnvironment* env)
    : GenericVideoFilter(_child), _restartlap(restart)
{
    const int depth = vi.BitsPerComponent();

    if (vi.IsRGB() || depth == 32 || !vi.IsPlanar())
        env->ThrowError("hqdn3d: clip must be in YUV 8..16-bit planar format.");
    if (LumSpac < 0.0 || LumSpac > 255.0)
        env->ThrowError("hqdn3d: ls must be between 0.0..255.0.");
    if (ChromSpac < 0.0 || ChromSpac > 255.0)
        env->ThrowError("hqdn3d: cs must be between 0.0..255.0.");
    if (LumTmp < 0.0 || LumTmp > 255.0)
        env->ThrowError("hqdn3d: lt must be between 0.0..255.0.");
    if (ChromTmp < 0.0 || ChromTmp > 255.0)
        env->ThrowError("hqdn3d: ct must be between 0.0..255.0.");
    if (_restartlap < 0)
        env->ThrowError("hqdn3d: restart must be non-negative value.");

    const int process_planes[3] = { y, u, v };
    for (int i = 0; i < 3; ++i)
    {
        switch (process_planes[i])
        {
            case 3: process[i] = 3; break;
            case 2: process[i] = 2; break;
            default: process[i] = 1; break;
        }
    }
    
    planecount = std::min(vi.NumComponents(), 3);
    const int planes[3] = { PLANAR_Y, PLANAR_U, PLANAR_V };

    for (int i = 0; i < planecount; ++i)
    {
        if (process[i] == 3)
        {
            line[i] = new uint16_t[((i) ? vi.width >> vi.GetPlaneWidthSubsampling(planes[i]) : vi.width) * vi.ComponentSize()];
            frame_prev[i] = new uint16_t[((i) ? (vi.width >> vi.GetPlaneWidthSubsampling(planes[i])) * (vi.height >> vi.GetPlaneHeightSubsampling(planes[i])) : vi.width * vi.height) * vi.ComponentSize()];
        }
        else
        {
            line[i] = nullptr;
            frame_prev[i] = nullptr;
        }
    }

    double strength[4] = { LumSpac = std::min(254.9, LumSpac),
    LumTmp = std::min(254.9, LumTmp),
    ChromSpac = std::min(254.9, ChromSpac),
    ChromTmp = std::min(254.9, ChromTmp) };

    for (int i = 0; i < 4; ++i)
    {
        coefs[i] = new int16_t[512 << LUT_BITS];
        precalc_coefs(strength[i], depth, coefs[i]);
    }       

    has_at_least_v8 = true;
    try { env->CheckVersion(8); }
    catch (const AvisynthError&) { has_at_least_v8 = false; };
}

hqdn3d::~hqdn3d()
{
    for (int i = 0; i < 4; ++i)
        delete[] coefs[i];

    for (int i = 0; i < planecount; ++i)
    {
        if (process[i] == 3)
        {
            delete[] frame_prev[i];
            delete[] line[i];
        }
    }
}

PVideoFrame __stdcall hqdn3d::GetFrame(int n, IScriptEnvironment* env)
{
    if (n == prev_frame)
        return cache;
    // if we skip some frames, filter the gap anyway
    if (n > prev_frame + 1 && n - prev_frame <= _restartlap + 1 && prev_frame >= 0)
    {
        for (int i = prev_frame + 1; i < n; ++i)
        {
            PVideoFrame src = child->GetFrame(i, env);
            filterFrame(src, env);
        }
    }
    // if processing out of sequence, filter several previous frames to minimize seeking problems
    else if (n == 0 || n != prev_frame + 1)
    {
        const int sn = std::max(0, n - _restartlap);
        PVideoFrame sf = child->GetFrame(sn, env);
        const int planes[3] = { PLANAR_Y, PLANAR_U, PLANAR_V };

        for (int i = 0; i < planecount; ++i)
        {

            const int sStride = sf->GetPitch(planes[i]);
            const int w = sf->GetRowSize(planes[i]);
            const int h = sf->GetHeight(planes[i]);
            const uint8_t* srcp = sf->GetReadPtr(planes[i]);

            for (int y = 0; y < h; ++y)
            {
                const uint8_t* src = srcp + y * sStride;
                uint16_t* dst = &frame_prev[0][y * w];

                for (int x = 0; x < w; ++x)
                    dst[x] = src[x] << 8;
            }
        }

        for (int i = sn + 1; i < n; ++i)
        {
            PVideoFrame src = child->GetFrame(i, env);
            filterFrame(src, env);
        }
    }
    
    prev_frame = n;
    PVideoFrame src = child->GetFrame(n, env);
    cache = filterFrame(src, env);

    return cache;
}

AVSValue __cdecl Create_hqdn3d(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    const float ls = args[1].AsFloatf(4.0f);
    const float cs = args[2].AsFloatf(3.0f * ls / 4.0f);
    const float lt = args[3].AsFloatf(6.0f * ls / 4.0f);
    const float ct = args[4].AsFloatf(lt * cs / ls);

    return new hqdn3d(
        args[0].AsClip(),
        ls,
        cs,
        lt,
        ct,
        args[5].AsInt(std::max(2LL, llrint(1.0 + std::max(lt, ct)))),
        args[6].AsInt(3),
        args[7].AsInt(3),
        args[8].AsInt(3),
        env);
}

const AVS_Linkage* AVS_linkage;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("hqdn3d", "c[ls]f[cs]f[lt]f[ct]f[restart]i[y]i[u]i[v]i", Create_hqdn3d, 0);
    return 0;
}

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
#include <memory>
#include <vector>
#include <execution>

#include "avisynth.h"

class hqdn3d : public GenericVideoFilter
{
    std::unique_ptr<int16_t[]> coefs[4];
    std::unique_ptr<uint16_t[]> line[3];
    std::unique_ptr<uint16_t[]> frame_prev[3];
    int process[3];
    bool has_at_least_v8;
    int restartlap;
    int prev_frame;
    PVideoFrame cache;

    template <typename T, int depth, int LUT_BITS, bool mt>
    PVideoFrame filterFrame(PVideoFrame src, IScriptEnvironment* env);

    PVideoFrame(hqdn3d::* filter)(PVideoFrame src, IScriptEnvironment* env);

public:
    hqdn3d(PClip _child, double LumSpac, double ChromSpac, double LumTmp, double ChromTmp, int restart_, int y, int u, int v, bool mt, IScriptEnvironment* env);
    int __stdcall SetCacheHints(int cachehints, int frame_range) override
    {
        return cachehints == CACHE_GET_MTMODE ? MT_SERIALIZED : 0;
    }
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override;
};

template <int LUT_BITS>
AVS_FORCEINLINE uint32_t lowpass(int prev, int cur, int16_t* coef)
{
    return cur + coef[(prev - cur) >> (8 - LUT_BITS)];
}

template <typename T, int depth, int LUT_BITS>
AVS_FORCEINLINE void denoise_temporal(const T* src, T* dst, uint16_t* frame_ant, int w, int h, int sstride, int dstride, int16_t* temporal)
{
    temporal += 256 << LUT_BITS;

    for (int y{ 0 }; y < h; ++y)
    {
        for (int x{ 0 }; x < w; ++x)
        {
            uint32_t tmp;
            frame_ant[x] = tmp = lowpass<LUT_BITS>(frame_ant[x], (src[x] << (16 - depth)) + (((1 << (16 - depth)) - 1) >> 1), temporal);
            dst[x] = tmp >> (16 - depth);
        }

        src += sstride;
        dst += dstride;
        frame_ant += w;
    }
}

template <typename T, int depth, int LUT_BITS>
AVS_FORCEINLINE void denoise_spatial(const T* src, T* dst, uint16_t* line_ant, uint16_t* frame_ant, int w, int h, int sstride, int dstride, int16_t* spatial, int16_t* temporal)
{
    uint32_t tmp;

    spatial += 256 << LUT_BITS;
    temporal += 256 << LUT_BITS;

    /* First line has no top neighbor. Only left one for each tmp and
     * last frame */
    int pixel_ant{ (src[0] << (16 - depth)) + (((1 << (16 - depth)) - 1) >> 1) };
    for (int x{ 0 }; x < w; ++x)
    {
        line_ant[x] = tmp = pixel_ant = lowpass<LUT_BITS>(pixel_ant, (src[x] << (16 - depth)) + (((1 << (16 - depth)) - 1) >> 1), spatial);
        frame_ant[x] = tmp = lowpass<LUT_BITS>(frame_ant[x], tmp, temporal);
        dst[x] = tmp >> (16 - depth);
    }

    for (int y{ 1 }; y < h; ++y)
    {
        src += sstride;
        dst += dstride;
        frame_ant += w;

        int x;
        pixel_ant = (src[0] << (16 - depth)) + (((1 << (16 - depth)) - 1) >> 1);
        for (x = 0; x < w - 1; ++x)
        {
            line_ant[x] = tmp = lowpass<LUT_BITS>(line_ant[x], pixel_ant, spatial);
            pixel_ant = lowpass<LUT_BITS>(pixel_ant, (src[x + 1] << (16 - depth)) + (((1 << (16 - depth)) - 1) >> 1), spatial);
            frame_ant[x] = tmp = lowpass<LUT_BITS>(frame_ant[x], tmp, temporal);
            dst[x] = tmp >> (16 - depth);
        }
        line_ant[x] = tmp = lowpass<LUT_BITS>(line_ant[x], pixel_ant, spatial);
        frame_ant[x] = tmp = lowpass<LUT_BITS>(frame_ant[x], tmp, temporal);
        dst[x] = tmp >> (16 - depth);
    }
}

template <typename T, int depth, int LUT_BITS>
AVS_FORCEINLINE void denoise(const T* src, T* dst, uint16_t* line_ant, uint16_t* frame_ant_ptr, int w, int h, int sstride, int dstride, int16_t* spatial, int16_t* temporal)
{
    if (spatial[0])
        denoise_spatial<T, depth, LUT_BITS>(src, dst, line_ant, frame_ant_ptr, w, h, sstride, dstride, spatial, temporal);
    else
        denoise_temporal<T, depth, LUT_BITS>(src, dst, frame_ant_ptr, w, h, sstride, dstride, temporal);
}

template <typename T, int depth, int LUT_BITS, bool mt>
PVideoFrame hqdn3d::filterFrame(PVideoFrame src, IScriptEnvironment* env)
{
    PVideoFrame dst{ (has_at_least_v8) ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi) };
    const int planes[3]{ PLANAR_Y, PLANAR_U, PLANAR_V };
    const int planecount{ std::min(vi.NumComponents(), 3) };

    auto loop = [&](int i)
    {
        const int height{ src->GetHeight(planes[i]) };

        if (process[i] == 3)
            denoise<T, depth, LUT_BITS>(reinterpret_cast<const T*>(src->GetReadPtr(planes[i])), reinterpret_cast<T*>(dst->GetWritePtr(planes[i])), line[i].get(), frame_prev[i].get(), src->GetRowSize(planes[i]) / sizeof(T),
                height, src->GetPitch(planes[i]) / sizeof(T), dst->GetPitch(planes[i]) / sizeof(T), coefs[(i) ? 2 : 0].get(), coefs[(i) ? 3 : 1].get());
        else if (process[i] == 2)
            env->BitBlt(dst->GetWritePtr(planes[i]), dst->GetPitch(planes[i]), src->GetReadPtr(planes[i]), src->GetPitch(planes[i]), src->GetRowSize(planes[i]), height);
    };

    if constexpr (mt)
    {
        std::vector<int> l(planecount);
        std::iota(std::begin(l), std::end(l), 0);
        std::for_each(std::execution::par, std::begin(l), std::end(l), loop);
    }
    else
    {
        for (intptr_t i = 0; i < planecount; ++i)
            loop(i);
    }

    return dst;
}

template <int LUT_BITS>
static void precalc_coefs(double dist25, int16_t* ct)
{
    const double gamma{ log(0.25) / log(1.0 - std::min(dist25, 252.0) / 255.0 - 0.00001) };

    for (int i{ -(256 << LUT_BITS) }; i < 256 << LUT_BITS; ++i)
    {
        const double f{ ((i << (9 - LUT_BITS)) + (1 << (8 - LUT_BITS)) - 1) / 512.0 }; // midpoint of the bin
        ct[(256 << LUT_BITS) + i] = static_cast<int16_t>(lrint(pow(std::max(0.0, 1.0 - fabs(f) / 255.0), gamma) * 256.0 * f));
    }

    ct[0] = !!dist25;
}

hqdn3d::hqdn3d(PClip _child, double LumSpac, double ChromSpac, double LumTmp, double ChromTmp, int restart_, int y, int u, int v, bool mt, IScriptEnvironment* env)
    : GenericVideoFilter(_child), restartlap(restart_)
{
    const int comp_size{ vi.ComponentSize() };

    if (vi.IsRGB() || comp_size == 4 || !vi.IsPlanar())
        env->ThrowError("hqdn3d: clip must be in YUV 8..16-bit planar format.");
    if (LumSpac < 0.0 || LumSpac > 255.0)
        env->ThrowError("hqdn3d: ls must be between 0.0..255.0.");
    if (ChromSpac < 0.0 || ChromSpac > 255.0)
        env->ThrowError("hqdn3d: cs must be between 0.0..255.0.");
    if (LumTmp < 0.0 || LumTmp > 255.0)
        env->ThrowError("hqdn3d: lt must be between 0.0..255.0.");
    if (ChromTmp < 0.0 || ChromTmp > 255.0)
        env->ThrowError("hqdn3d: ct must be between 0.0..255.0.");
    if (restartlap < 0)
        env->ThrowError("hqdn3d: restart must be non-negative value.");

    const int process_planes[3]{ y, u, v };
    for (int i = 0; i < 3; ++i)
    {
        switch (process_planes[i])
        {
            case 3: process[i] = 3; break;
            case 2: process[i] = 2; break;
            default: process[i] = 1; break;
        }
    }

    const int planecount{ std::min(vi.NumComponents(), 3) };

    if (planecount > 1)
    {
        const int cw{ (process[1] == 3 || process[2] == 3) ? (vi.width >> vi.GetPlaneWidthSubsampling(PLANAR_U)) : 0 };
        const int ch{ (process[1] == 3 || process[2] == 3) ? (vi.height >> vi.GetPlaneHeightSubsampling(PLANAR_U)) : 0 };

        for (int i{ 0 }; i < planecount; ++i)
        {
            if (process[i] == 3)
            {
                line[i] = std::make_unique<uint16_t[]>(((i) ? cw : vi.width) * comp_size);
                frame_prev[i] = std::make_unique<uint16_t[]>(((i) ? (cw * ch) : (vi.width * vi.height)) * comp_size);
            }
        }
    }
    else
    {
        line[0] = std::make_unique<uint16_t[]>(vi.width * comp_size);
        frame_prev[0] = std::make_unique<uint16_t[]>(vi.width * vi.height * comp_size);
    }

    prev_frame = -99999;
    const double strength[4]{ std::min(254.9, LumSpac), std::min(254.9, LumTmp), std::min(254.9, ChromSpac), std::min(254.9, ChromTmp) };

    switch (comp_size)
    {
        case 2:
        {
            for (int i{ 0 }; i < 4; ++i)
            {
                coefs[i] = std::make_unique<int16_t[]>(512 << 8);
                precalc_coefs<8>(strength[i], coefs[i].get());
            }

            if (mt)
            {
                switch (vi.BitsPerComponent())
                {
                    case 16: filter = &hqdn3d::filterFrame<uint16_t, 16, 8, true>; break;
                    case 14: filter = &hqdn3d::filterFrame<uint16_t, 14, 8, true>; break;
                    case 12: filter = &hqdn3d::filterFrame<uint16_t, 12, 8, true>; break;
                    case 10: filter = &hqdn3d::filterFrame<uint16_t, 10, 8, true>; break;
                }
            }
            else
            {
                switch (vi.BitsPerComponent())
                {
                    case 16: filter = &hqdn3d::filterFrame<uint16_t, 16, 8, false>; break;
                    case 14: filter = &hqdn3d::filterFrame<uint16_t, 14, 8, false>; break;
                    case 12: filter = &hqdn3d::filterFrame<uint16_t, 12, 8, false>; break;
                    case 10: filter = &hqdn3d::filterFrame<uint16_t, 10, 8, false>; break;
                }
            }
            break;
        }
        default:
        {
            for (int i{ 0 }; i < 4; ++i)
            {
                coefs[i] = std::make_unique<int16_t[]>(512 << 4);
                precalc_coefs<4>(strength[i], coefs[i].get());
            }

            if (mt)
                filter = &hqdn3d::filterFrame<uint8_t, 8, 4, true>;
            else
                filter = &hqdn3d::filterFrame<uint8_t, 8, 4, false>;
            break;
        }
    }

    has_at_least_v8 = true;
    try { env->CheckVersion(8); }
    catch (const AvisynthError&) { has_at_least_v8 = false; };
}

PVideoFrame __stdcall hqdn3d::GetFrame(int n, IScriptEnvironment* env)
{
    if (n == prev_frame)
        return cache;
    // if we skip some frames, filter the gap anyway
    if (n > prev_frame + 1 && n - prev_frame <= restartlap + 1 && prev_frame >= 0)
    {
        for (int i{ prev_frame + 1 }; i < n; ++i)
            (this->*filter)(child->GetFrame(i, env), env);
    }
    // if processing out of sequence, filter several previous frames to minimize seeking problems
    else if (n == 0 || n != prev_frame + 1)
    {
        const int sn{ std::max(0, n - restartlap) };
        PVideoFrame sf{ child->GetFrame(sn, env) };
        const int planes[3]{ PLANAR_Y, PLANAR_U, PLANAR_V };
        const int planecount{ std::min(vi.NumComponents(), 3) };

        if (vi.ComponentSize() == 1)
        {
            for (int i{ 0 }; i < planecount; ++i)
            {
                if (process[i] == 3)
                {
                    const int sStride{ sf->GetPitch(planes[i]) };
                    const int w{ sf->GetRowSize(planes[i]) };
                    const int h{ sf->GetHeight(planes[i]) };
                    const uint8_t* srcp{ sf->GetReadPtr(planes[i]) };

                    for (int y{ 0 }; y < h; ++y)
                    {
                        const uint8_t* src{ srcp + y * sStride };
                        uint16_t* dst{ &frame_prev[i][y * w] };

                        for (int x{ 0 }; x < w; ++x)
                            dst[x] = src[x] << 8;
                    }
                }
            }
        }
        else
        {
            for (int i{ 0 }; i < planecount; ++i)
            {
                if (process[i] == 3)
                {
                    const int sStride{ sf->GetPitch(planes[i]) / vi.ComponentSize() };
                    const int w{ sf->GetRowSize(planes[i]) / vi.ComponentSize() };
                    const int h{ sf->GetHeight(planes[i]) };
                    const uint16_t* srcp{ reinterpret_cast<const uint16_t*>(sf->GetReadPtr(planes[i])) };

                    for (int y{ 0 }; y < h; ++y)
                    {
                        const uint16_t* src{ srcp + y * sStride };
                        uint16_t* dst{ &frame_prev[i][y * w] };

                        for (int x{ 0 }; x < w; ++x)
                            dst[x] = src[x] << (16 - vi.BitsPerComponent());
                    }
                }
            }
        }

        for (int i{ sn + 1 }; i < n; ++i)
            (this->*filter)(child->GetFrame(i, env), env);
    }

    prev_frame = n;
    cache = (this->*filter)(child->GetFrame(n, env), env);

    return cache;
}

AVSValue __cdecl Create_hqdn3d(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    const double ls{ args[1].AsFloat(4.0f) };
    const double cs{ args[2].AsFloat(static_cast<float>(3.0f * ls / 4.0f)) };
    const double lt{ args[3].AsFloat(static_cast<float>(6.0f * ls / 4.0f)) };
    const double ct{ args[4].AsFloat(static_cast<float>(lt * cs / ls)) };

    return new hqdn3d(
        args[0].AsClip(),
        ls,
        cs,
        lt,
        ct,
        args[5].AsInt(std::max(2, static_cast<int>(llrint(1.0 + std::max(lt, ct))))),
        args[6].AsInt(3),
        args[7].AsInt(3),
        args[8].AsInt(3),
        args[9].AsBool(false),
        env);
}

const AVS_Linkage* AVS_linkage;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("hqdn3d", "c[ls]f[cs]f[lt]f[ct]f[restart]i[y]i[u]i[v]i[mt]b", Create_hqdn3d, 0);
    return 0;
}

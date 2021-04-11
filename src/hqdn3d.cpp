/*
    HQDN3D 0.11 for Avisynth

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

#include "internal.h"


static inline unsigned int LowPassMul(unsigned int PrevMul, unsigned int CurrMul, int* Coef){
//	int dMul= (PrevMul&0xFFFFFF)-(CurrMul&0xFFFFFF);
	int dMul= PrevMul-CurrMul;
	int d=((dMul+0x10007FF)/(65536/16));
	return CurrMul + Coef[d];
}

static void deNoise(const unsigned char *Frame,  // mpi->planes[x]
                    unsigned char *FrameDest,    // dmpi->planes[x]
                    unsigned int *LineAnt,       // vf->priv->Line (width bytes)
                    unsigned short *FrameAnt,
                    int W, int H, int sStride, int dStride,
                    int *Horizontal, int *Vertical, int *Temporal)
{
	int X, Y;
	int sLineOffs = 0, dLineOffs = 0;
	unsigned int PixelAnt;
	int PixelDst;

	/* First pixel has no left nor top neightbour. Only previous frame */
	LineAnt[0] = PixelAnt = Frame[0]<<16;
	PixelDst = LowPassMul(FrameAnt[0]<<8, PixelAnt, Temporal);
	FrameAnt[0] = ((PixelDst+0x1000007F)/256);
	FrameDest[0]= ((PixelDst+0x10007FFF)/65536);

	/* Fist line has no top neightbour. Only left one for each pixel and
	 * last frame */
	for (X = 1; X < W; X++){
		LineAnt[X] = PixelAnt = LowPassMul(PixelAnt, Frame[X]<<16, Horizontal);
		PixelDst = LowPassMul(FrameAnt[X]<<8, PixelAnt, Temporal);
		FrameAnt[X] = ((PixelDst+0x1000007F)/256);
		FrameDest[X]= ((PixelDst+0x10007FFF)/65536);
	}

	for (Y = 1; Y < H; Y++){
		unsigned int PixelAnt;
		unsigned short* LinePrev=&FrameAnt[Y*W];
		sLineOffs += sStride, dLineOffs += dStride;
		/* First pixel on each line doesn't have previous pixel */
		PixelAnt = Frame[sLineOffs]<<16;
		LineAnt[0] = LowPassMul(LineAnt[0], PixelAnt, Vertical);
		PixelDst = LowPassMul(LinePrev[0]<<8, LineAnt[0], Temporal);
		LinePrev[0] = ((PixelDst+0x1000007F)/256);
		FrameDest[dLineOffs]= ((PixelDst+0x10007FFF)/65536);

		for (X = 1; X < W; X++){
			int PixelDst;
			/* The rest are normal */
			PixelAnt = LowPassMul(PixelAnt, Frame[sLineOffs+X]<<16, Horizontal);
			LineAnt[X] = LowPassMul(LineAnt[X], PixelAnt, Vertical);
			PixelDst = LowPassMul(LinePrev[X]<<8, LineAnt[X], Temporal);
			LinePrev[X] = ((PixelDst+0x1000007F)/256);
			FrameDest[dLineOffs+X]= ((PixelDst+0x10007FFF)/65536);
		}
	}
}

#define ABS(A) ( (A) > 0 ? (A) : -(A) )

static void PrecalcCoefs(int *Ct, double Dist25)
{
	int i;
	double Gamma, Simil, C;

	Gamma = log(0.25) / log(1.0 - Dist25/255.0 - 0.00001);

	for (i = -255*16; i < 256*16; i++)
	{
		Simil = 1.0 - ABS(i) / (16*255.0);
		C = pow(Simil, Gamma) * 65536.0 * (double)i / 16.0;
		Ct[16*256+i] = (int)((C<0) ? (C-0.5) : (C+0.5));
	}
}


class hqdn3d : public GenericVideoFilter
{
public:
	hqdn3d(PClip _child, double LumSpac, double ChromSpac, double LumTmp, double ChromTmp,
		int RestartLap, IScriptEnvironment* _env) :
		GenericVideoFilter(_child), restart_lap(RestartLap), env(_env)
	{
		if(!vi.IsYV12())
			env->ThrowError("hqdn3d: requires YV12 source");

		if(LumSpac < 0)
			LumSpac = 4.0;
		if(ChromSpac < 0)
			ChromSpac = .75 * LumSpac;
		if(LumTmp < 0)
			LumTmp = 1.5 * LumSpac;
		if(ChromTmp < 0) {
			if(LumSpac == 0)
				ChromTmp = ChromSpac * 1.5;
			else
				ChromTmp = LumTmp * ChromSpac / LumSpac;
		}
		LumSpac = __min(254.9, LumSpac);
		ChromSpac = __min(254.9, ChromSpac);
		LumTmp = __min(254.9, LumTmp);
		ChromTmp = __min(254.9, ChromTmp);

		if(restart_lap < 0)
			restart_lap = __max(2, (int)(1 + __max(LumTmp, ChromTmp)));

        PrecalcCoefs(Coefs[0], LumSpac);
        PrecalcCoefs(Coefs[1], LumTmp);
        PrecalcCoefs(Coefs[2], ChromSpac);
        PrecalcCoefs(Coefs[3], ChromTmp);

		prev_frame = -99999;
		w= vi.width;
		h= vi.height;
		cw = w>>1;
		ch = h>>1;

		Line = new unsigned int[w];
		Frame[0] = new unsigned short[w*h];
		Frame[1] = new unsigned short[cw*ch];
		Frame[2] = new unsigned short[cw*ch];
	}
	~hqdn3d()
	{
		if(Line)     delete [] Line;
		if(Frame[0]) delete [] Frame[0];
		if(Frame[1]) delete [] Frame[1];
		if(Frame[2]) delete [] Frame[2];
	}

	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* _env);

private:
	//// Options
	int restart_lap; // how many frames back we look when seeking

	//// State
	IScriptEnvironment* env;
	// TODO: cache more than one frame, to speed up playing/seeking backwards
	PVideoFrame cache;

	int Coefs[4][512*16];
	unsigned int *Line;
	unsigned short *Frame[3];

	int prev_frame;
	int w, cw, h, ch;

	PVideoFrame filterFrame(PVideoFrame cur);
};

PVideoFrame hqdn3d::filterFrame(PVideoFrame cur)
{
	PVideoFrame newframe = env->NewVideoFrame(vi);

	deNoise(cur->GetReadPtr(PLANAR_Y),
			newframe->GetWritePtr(PLANAR_Y),
			Line, Frame[0], w, h,
			cur->GetPitch(PLANAR_Y),
			newframe->GetPitch(PLANAR_Y),
			Coefs[0], Coefs[0], Coefs[1]);
	deNoise(cur->GetReadPtr(PLANAR_U),
			newframe->GetWritePtr(PLANAR_U),
			Line, Frame[1], cw, ch,
			cur->GetPitch(PLANAR_U),
			newframe->GetPitch(PLANAR_U),
			Coefs[2], Coefs[2], Coefs[3]);
	deNoise(cur->GetReadPtr(PLANAR_V),
			newframe->GetWritePtr(PLANAR_V),
			Line, Frame[2], cw, ch,
			cur->GetPitch(PLANAR_V),
			newframe->GetPitch(PLANAR_V),
			Coefs[2], Coefs[2], Coefs[3]);

	return newframe;
}

PVideoFrame __stdcall hqdn3d::GetFrame(int n, IScriptEnvironment* _env)
{
	env = _env;

	if(n == prev_frame)
		return cache;
	// if we skip some frames, filter the gap anyway
	else if(n > prev_frame+1 && n - prev_frame <= restart_lap+1 && prev_frame >= 0) {
		for(int i=prev_frame+1; i<n; i++)
			filterFrame(child->GetFrame(i, env));
	}
	// if processing out of sequence, filter several previous frames to minimize seeking problems
	else if(n == 0 || n != prev_frame+1) {
		int sn = __max(0, n - restart_lap);
		PVideoFrame sf = child->GetFrame(sn, env);
		int x, y, c;
		int sStride = sf->GetPitch(PLANAR_Y);
		const BYTE* srcp = sf->GetReadPtr(PLANAR_Y);
		for(y = 0; y < h; y++) {
			unsigned short* dst=&Frame[0][y*w];
			const BYTE* src=srcp+y*sStride;
			for(x = 0; x < w; x++) dst[x]=src[x]<<8;
		}
		for(c=1; c<=2; c++) {
			sStride = sf->GetPitch(PLANAR_U);
			srcp = sf->GetReadPtr((c==1) ? PLANAR_U : PLANAR_V);
			for(y = 0; y < ch; y++) {
				unsigned short* dst=&Frame[c][y*cw];
				const BYTE* src=srcp+y*sStride;
				for(x = 0; x < cw; x++) dst[x]=src[x]<<8;
			}
		}
		for(int i=sn+1; i<n; i++)
			filterFrame(child->GetFrame(i, env));
	}

	prev_frame = n;
	cache = filterFrame(child->GetFrame(n, env));
	return cache;
}

AVSValue __cdecl Create_hqdn3d(AVSValue args, void* user_data, IScriptEnvironment* env)
{
	return new hqdn3d(args[0].AsClip(),
		args[1].AsFloat(-1),
		args[2].AsFloat(-1),
		args[3].AsFloat(-1),
		args[4].AsFloat(-1),
		args[5].AsInt(-1),
		env);
}

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit2(IScriptEnvironment* env)
{
    env->AddFunction("hqdn3d", "c[ls]f[cs]f[lt]f[ct]f[restart]i", Create_hqdn3d, 0);
    return 0;
}

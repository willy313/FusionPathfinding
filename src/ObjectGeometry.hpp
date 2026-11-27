#pragma once
// Geometry of Fusion objects used by pathfinding (Windows only: needs the Fusion runtime headers).
#ifdef _WIN32
#include <cmath>
#include <algorithm>

// Bounding rectangle of an object in frame coordinates.
inline bool ObjectRect(RunHeader* rhPtr, HeaderObject* ho, float& left, float& top, float& right, float& bottom)
{
	// Rectangle A: position + hotspot + image size. Frame coordinates, always valid, ignores scale/angle.
	left = (float)(ho->get_X() - ho->get_ImgXSpot());
	top = (float)(ho->get_Y() - ho->get_ImgYSpot());
	right = left + (float)ho->get_ImgWidth();
	bottom = top + (float)ho->get_ImgHeight();

	// Rectangle B: hoRect is the on-screen display rectangle in WINDOW coordinates and does account for
	// scale/angle. Convert to frame coordinates and use it only if it agrees with A (same centre, within
	// the object's size); otherwise trust A.
	if (ho->hoRect.right > ho->hoRect.left && ho->hoRect.bottom > ho->hoRect.top)
	{
		const float bl = (float)(ho->hoRect.left + rhPtr->get_WindowX()), bt = (float)(ho->hoRect.top + rhPtr->get_WindowY());
		const float br = (float)(ho->hoRect.right + rhPtr->get_WindowX()), bb = (float)(ho->hoRect.bottom + rhPtr->get_WindowY());
		const float acx = (left + right) * 0.5f, acy = (top + bottom) * 0.5f;
		const float bcx = (bl + br) * 0.5f, bcy = (bt + bb) * 0.5f;
		const float tolX = (std::max)(right - left, br - bl), tolY = (std::max)(bottom - top, bb - bt);
		if (std::fabs(acx - bcx) <= tolX * 0.5f && std::fabs(acy - bcy) <= tolY * 0.5f)
		{
			left = bl; top = bt; right = br; bottom = bb;
		}
	}
	return right > left && bottom > top;
}

// Radius pathfinding uses for an object: the distance from its hotspot to the farthest corner of its image,
// so the whole body fits wherever the hotspot goes.
inline float RadiusOfHeader(HeaderObject* ho)
{
	const float w = (float)ho->get_ImgWidth(), h = (float)ho->get_ImgHeight();
	const float xs = (float)ho->get_ImgXSpot(), ys = (float)ho->get_ImgYSpot();
	const float dx = (std::max)(xs, w - xs), dy = (std::max)(ys, h - ys);
	return (w > 0.f && h > 0.f) ? std::sqrt(dx * dx + dy * dy) : 0.f;
}

// Pathfinding radius with the "size percent" setting applied (100 = the object's own size).
inline float PathRadius(HeaderObject* ho, int percent)
{
	return RadiusOfHeader(ho) * (float)(std::max)(percent, 1) / 100.f;
}

// The largest circle around the hotspot that fits inside the object's image: the default personal space.
inline float InscribedRadius(HeaderObject* ho)
{
	const float w = (float)ho->get_ImgWidth(), h = (float)ho->get_ImgHeight();
	const float xs = (float)ho->get_ImgXSpot(), ys = (float)ho->get_ImgYSpot();
	if (w <= 0.f || h <= 0.f) return 8.f;
	return (std::max)(1.f, (std::min)((std::min)(xs, w - xs), (std::min)(ys, h - ys)));
}
#endif

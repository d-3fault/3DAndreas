#include "routeline.hpp"

#include <algorithm>
#include <cmath>

namespace physics {

namespace {

constexpr float SamePlace = 0.05f;
constexpr float MostDotWidths = 4.0f;

RoutePoint pointAt(const std::vector<RoutePoint>& points, const std::vector<float>& along, float distance) {
	auto after = std::upper_bound(along.begin(), along.end(), distance);
	if (after == along.begin()) {
		return points.front();
	}
	if (after == along.end()) {
		return points.back();
	}
	size_t index = size_t(after - along.begin()) - 1;
	float span = along[index + 1] - along[index];
	float t = span > 0.0f ? (distance - along[index]) / span : 0.0f;
	const RoutePoint& a = points[index];
	const RoutePoint& b = points[index + 1];
	return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
}

}

bool lineBefore(const LineSquare& a, const LineSquare& b) {
	return a.key != b.key ? a.key < b.key : a.size < b.size;
}

void planLine(const std::vector<RoutePoint>& points, const std::vector<float>& along, float position, float width, int budget, std::vector<LineSquare>& squares) {
	squares.clear();
	if (points.empty() || along.size() != points.size() || !(width > 0.0f) || budget <= 0) {
		return;
	}
	float length = along.back();
	float remaining = std::max(length - std::max(position, 0.0f), 0.0f);
	int64_t start = int64_t(std::floor(remaining / width));
	int64_t line = start + 1 <= budget ? start + 1 : std::max<int64_t>(budget / 2, 1);
	int64_t end = start - line + 1;
	auto add = [&](int64_t key, float size) {
		RoutePoint at = pointAt(points, along, length - float(key) * width);
		squares.push_back({ key, size, at.x, at.y, -1 });
	};
	int64_t left = budget - line;
	if (end > 0 && left > 0) {
		int64_t stride = 2;
		while (stride * left < end) {
			stride *= 2;
		}
		float size = std::min(std::max(width, float(stride) * width * 0.5f), width * MostDotWidths);
		for (int64_t key = 0; key < end; key += stride) {
			add(key, size);
		}
	}
	for (int64_t key = end; key <= start; ++key) {
		add(key, width);
	}
	std::sort(squares.begin(), squares.end(), lineBefore);
}

LineChange compareLine(const std::vector<LineSquare>& shown, const std::vector<LineSquare>& wanted) {
	LineChange change;
	size_t at = 0;
	for (const LineSquare& square : shown) {
		while (at != wanted.size() && lineBefore(wanted[at], square)) {
			change.added.push_back(wanted[at++]);
		}
		if (at != wanted.size() && !lineBefore(square, wanted[at]) && std::fabs(wanted[at].x - square.x) <= SamePlace && std::fabs(wanted[at].y - square.y) <= SamePlace) {
			change.kept.push_back(square);
			++at;
			continue;
		}
		change.hidden.push_back(square);
	}
	while (at != wanted.size()) {
		change.added.push_back(wanted[at++]);
	}
	return change;
}

}

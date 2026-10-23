#include "../src/routeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <set>
#include <utility>
#include <vector>

using namespace physics;

namespace {

int checks = 0;
int failures = 0;

void check(bool ok, const char* what) {
	++checks;
	if (!ok) {
		++failures;
		std::printf("FAIL %s\n", what);
	}
}

struct Line {
	std::vector<RoutePoint> points;
	std::vector<float> along;
};

Line makeLine(std::vector<RoutePoint> points) {
	Line line;
	line.points = std::move(points);
	line.along.assign(line.points.size(), 0.0f);
	for (size_t index = 1; index < line.points.size(); ++index) {
		const RoutePoint& a = line.points[index - 1];
		const RoutePoint& b = line.points[index];
		line.along[index] = line.along[index - 1] + std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
	}
	return line;
}

std::vector<LineSquare> plan(const Line& line, float position, float width, int budget) {
	std::vector<LineSquare> squares;
	planLine(line.points, line.along, position, width, budget, squares);
	return squares;
}

float gap(const LineSquare& a, const LineSquare& b) {
	return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

size_t countSize(const std::vector<LineSquare>& squares, float size) {
	size_t count = 0;
	for (const LineSquare& square : squares) {
		count += square.size == size ? 1 : 0;
	}
	return count;
}

void testWhole() {
	Line line = makeLine({ { 0.0f, 0.0f, 0.0f }, { 1000.0f, 0.0f, 0.0f } });
	std::vector<LineSquare> squares = plan(line, 0.0f, 8.0f, 512);
	check(squares.size() == 126, "a kilometre at 8 metres is 126 squares");
	check(countSize(squares, 8.0f) == squares.size(), "a line that fits has no dots");
	bool steady = true;
	for (size_t index = 1; index < squares.size(); ++index) {
		steady = steady && std::fabs(gap(squares[index - 1], squares[index]) - 8.0f) < 0.01f;
	}
	check(steady, "the squares of a straight line are one width apart");
	check(squares.front().key == 0 && std::fabs(squares.front().x - 1000.0f) < 0.01f, "the first key is the end");
	check(std::fabs(squares.back().x) < 0.01f, "the last key is where the player is");
}

void testLong() {
	Line line = makeLine({ { 0.0f, 0.0f, 0.0f }, { 10000.0f, 0.0f, 0.0f } });
	std::vector<LineSquare> squares = plan(line, 0.0f, 8.0f, 128);
	check(squares.size() <= 128, "a long line keeps to the budget");
	check(countSize(squares, 8.0f) == 64, "half the budget draws the way ahead");
	check(squares.front().key == 0, "the dots reach the end");
	bool small = true;
	for (const LineSquare& square : squares) {
		small = small && square.size <= 32.0f;
	}
	check(small, "dots are four widths at most");
	std::vector<LineSquare> wide = plan(line, 0.0f, 8.0f, 512);
	check(countSize(wide, 8.0f) == 256 && wide.size() <= 512, "512 squares draw two kilometres ahead");
}

void testMoving() {
	Line line = makeLine({ { 0.0f, 0.0f, 0.0f }, { 10000.0f, 0.0f, 0.0f } });
	LineChange same = compareLine(plan(line, 2.0f, 8.0f, 128), plan(line, 5.0f, 8.0f, 128));
	check(same.hidden.empty() && same.added.empty(), "moving inside a square changes nothing");
	LineChange step = compareLine(plan(line, 2.0f, 8.0f, 128), plan(line, 10.0f, 8.0f, 128));
	check(step.hidden.size() == 1 && step.added.size() == 1, "moving a square hides one behind and shows one ahead");
	Line shortLine = makeLine({ { 0.0f, 0.0f, 0.0f }, { 1000.0f, 0.0f, 0.0f } });
	LineChange whole = compareLine(plan(shortLine, 2.0f, 8.0f, 512), plan(shortLine, 10.0f, 8.0f, 512));
	check(whole.hidden.size() == 1 && whole.added.empty(), "a whole line only loses the square behind");
	size_t sent = 0;
	std::vector<LineSquare> shown = plan(line, 0.0f, 8.0f, 128);
	for (float position = 1.0f; position < 10000.0f; position += 7.0f) {
		std::vector<LineSquare> wanted = plan(line, position, 8.0f, 128);
		LineChange change = compareLine(shown, wanted);
		sent += change.hidden.size() + change.added.size();
		shown = std::move(change.kept);
		shown.insert(shown.end(), change.added.begin(), change.added.end());
		std::sort(shown.begin(), shown.end(), lineBefore);
	}
	std::printf("driving 10 km in steps of 7 m with 128 squares sends %zu gang zone changes\n", sent);
	check(sent < 4000, "driving 10 km sends a few thousand changes, not the line at every step");
}

void testReroute() {
	Line before = makeLine({ { 0.0f, 500.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 1000.0f, 0.0f, 0.0f } });
	Line after = makeLine({ { 200.0f, 300.0f, 0.0f }, { 300.0f, 0.0f, 0.0f }, { 1000.0f, 0.0f, 0.0f } });
	std::vector<LineSquare> shown = plan(before, 0.0f, 8.0f, 512);
	std::vector<LineSquare> wanted = plan(after, 0.0f, 8.0f, 512);
	LineChange change = compareLine(shown, wanted);
	std::printf("route found again: %zu kept, %zu hidden, %zu shown, of %zu and %zu\n", change.kept.size(), change.hidden.size(), change.added.size(), shown.size(), wanted.size());
	check(change.kept.size() == 88, "the 700 metres both routes end with keep their 88 squares");
	check(change.kept.size() + change.hidden.size() == shown.size(), "every square shown is kept or hidden");
	check(change.kept.size() + change.added.size() == wanted.size(), "every square wanted is kept or shown");
}

void testEdges() {
	Line line = makeLine({ { 0.0f, 0.0f, 0.0f }, { 1000.0f, 0.0f, 0.0f } });
	check(plan(line, 0.0f, 8.0f, 1).size() == 1, "a budget of one is one square");
	check(plan(line, 2000.0f, 8.0f, 512).size() == 1, "past the end only the end is left");
	check(plan(line, -50.0f, 8.0f, 512).size() == 126, "before the start counts from the start");
	check(plan(Line(), 0.0f, 8.0f, 512).empty(), "an empty route has no line");
	check(plan(line, 0.0f, 0.0f, 512).empty(), "a line of no width is not drawn");
	Line point = makeLine({ { 5.0f, 5.0f, 0.0f } });
	check(plan(point, 0.0f, 8.0f, 512).size() == 1, "a route of one point is one square");
}

void testRandom() {
	std::mt19937 random(7);
	std::uniform_real_distribution<float> coordinate(-1500.0f, 1500.0f);
	std::uniform_real_distribution<float> unit(0.0f, 1.0f);
	bool ordered = true;
	bool unique = true;
	bool bounded = true;
	bool joined = true;
	bool ended = true;
	for (int round = 0; round != 1000; ++round) {
		std::vector<RoutePoint> points;
		int count = 2 + int(unit(random) * 10.0f);
		for (int index = 0; index != count; ++index) {
			points.push_back({ coordinate(random), coordinate(random), 0.0f });
		}
		Line line = makeLine(points);
		float width = 4.0f + unit(random) * 16.0f;
		int budget = 1 + int(unit(random) * 511.0f);
		float position = unit(random) * line.along.back();
		std::vector<LineSquare> squares = plan(line, position, width, budget);
		bounded = bounded && !squares.empty() && int(squares.size()) <= budget;
		ended = ended && (budget < 2 || squares.front().key == 0);
		std::set<std::pair<long long, float>> seen;
		for (size_t index = 0; index != squares.size(); ++index) {
			unique = unique && seen.insert({ (long long)squares[index].key, squares[index].size }).second;
			if (index != 0) {
				ordered = ordered && !lineBefore(squares[index], squares[index - 1]);
				if (squares[index].size == width && squares[index - 1].size == width && squares[index].key == squares[index - 1].key + 1) {
					joined = joined && gap(squares[index], squares[index - 1]) <= width + 0.01f + line.along.back() * 2e-6f;
				}
			}
		}
	}
	check(ordered, "random lines come sorted");
	check(unique, "random lines have no square twice");
	check(bounded, "random lines keep to the budget");
	check(ended, "random lines mark the end when the budget has room for it");
	check(joined, "neighbouring squares of the line are no more than a width apart");
}

}

int main() {
	testWhole();
	testLong();
	testMoving();
	testReroute();
	testEdges();
	testRandom();
	std::printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}

#include <iostream>
#include "CImg.h";
#include <vector>
#include <random>
#include <set>
#include <stack>

using namespace std;
using namespace cimg_library;


const unsigned char white[]{ 255,255,255 };

// <x,y>
typedef pair<int, int> Point;
typedef pair<Point, Point> Line;

class ComputationalGeo {
public:

	cimg_library::CImg<unsigned char> img;
	CImgDisplay disp;
	const int width;
	const int height;

private:
	mt19937 mt;
public:
	ComputationalGeo() :img(cimg_library::CImg<unsigned char>(800, 800, 1, 1, 0)), width(img._width), height(img._height) {
		random_device rd;
		mt = mt19937(rd());
	}

	void run() {
		//runIntersect();
		// runSweepingSegment();
		runConvexHull();
	}

private:
	void runConvexHull() {
		disp = CImgDisplay(img, "Convex Hull");
		while (!disp.is_closed()) {
			disp.wait();
			if (disp.button() & 1) {
				img.fill(0);
				// Gen pts
				auto pts = genNPoints(35);

				// draw pts
				drawPoints(pts);

				// Get Convex Hull pts
				// auto convexHullPts = GrahamScan(pts);
				auto convexHullPts = Jarvis(pts);

				// Draw Convex Hull
				for (int j = 0; j < convexHullPts.size() - 1; j++)
					drawLine(convexHullPts[j], convexHullPts[j + 1]);
				drawLine(convexHullPts.back(), convexHullPts.front());

				// Display img
				disp.display(img);
			}
		}
	}

	/*
	Description:
	Find the Convex Hull using Graham Scan method:
	Find lowest pt
	Order the pt P by angle P-lowest-Xaxis
	Make sure the angle formed by points Next-to-top(s), top(s) and pt[i] make nonleft turn
	Return:
	ConvexHull Pts Coordinate
	*/
	vector<Point> GrahamScan(vector<Point> pts) {
		_ASSERT(pts.size() >= 3);
		vector<Point> cvxHull;

		/* Find lowest pt */
		Point lowest = pts[0];
		for (auto pt : pts) lowest = lowest.second < pt.second ? lowest : pt;

		/*Order the pt P by angle P-lowest-Xaxis*/
		sort(pts.begin(), pts.end(), [lowest](auto p1, auto p2) {
			if (p1 == lowest) return true;
			if (p2 == lowest) return false;

			double ang1 = 1.0f*(p1.first - lowest.first) / (p1.second - lowest.second),
				ang2 = 1.0f*(p2.first - lowest.first) / (p2.second - lowest.second);
			if (ang1 > ang2) return true;
			else if (ang1 < ang2) return false;
			else {
				double dis1 = (p1.first - lowest.first) * (p1.first - lowest.first) + (p1.second - lowest.second)*(p1.second - lowest.second),
					dis2 = (p2.first - lowest.first) * (p2.first - lowest.first) + (p2.second - lowest.second)*(p2.second - lowest.second);
				if (dis1 < dis2) return true;
				else return false;
			}
		});

		/*Make sure the angle formed by points Next-to-top(s), top(s) and pt[i] make nonleft turn.*/
		vector<int> s({ 0,1,2 });
		for (int i = 3; i < pts.size(); i++) {
			while (s.size() > 1 && clockWise(pts[s[s.size() - 2]], pts[s.back()], pts[i]) < 0)
				s.pop_back();
			s.push_back(i);
		}

		/* Return result */
		for (int i : s)
			cvxHull.push_back(pts[i]);

		return cvxHull;
	}

	vector<Point> Jarvis(vector<Point> pts) {
		_ASSERT(pts.size() >= 3);
		vector<Point> cvxHull;

		/* Find lowest pt */
		Point lowest(pts[0]), highest(pts[0]);
		for (auto pt : pts) {
			lowest = lowest.second < pt.second ? lowest : pt;
			highest = highest.second > pt.second ? highest : pt;
		}

		sort(pts.begin(), pts.end(), [](Point a, Point b) {return a.first < b.first; });
		auto lowestIt = lower_bound(pts.begin(), pts.end(), lowest, [](Point a, Point b) {
			if (a.first < b.first) return true;
			else if (a.first > b.first) return false;
			else return a.second < b.second;
		});

		// return pts.end() if not found.    
		auto findNextCvxHullPt = [this, &pts, &lowestIt](vector<Point>::const_iterator p, bool left) {
			//If lowestIt is the leftmost or rightmost, this method will not be called with the corresponding "left".
			vector<Point>::const_iterator nextPt = pts.begin();
			if (nextPt == p) nextPt++;
			bool found = false;
			if (left) {
				for (auto it = pts.begin(); it != pts.end(); it++) {
					int cmp = clockWise(*p, *it, *nextPt);
					if (cmp < 0 || (cmp == 0 && distance(*nextPt, *p) < distance(*it, *p))) nextPt = it;
				}
			}
			else {
				for (auto it = pts.begin(); it != pts.end(); it++) {
					int cmp = clockWise(*p, *it, *nextPt);
					if (cmp > 0 || (cmp == 0 && distance(*nextPt, *p) < distance(*it, *p))) nextPt = it;
				}
			}
			if (nextPt == p) nextPt = pts.end();
			return nextPt;
		};

		cvxHull.push_back(lowest);
		// right
		auto nextPt = findNextCvxHullPt(lowestIt, false);
		while (nextPt->second != highest.second) {
			cvxHull.push_back(*nextPt);
			nextPt = findNextCvxHullPt(nextPt, false);
		}
		cvxHull.push_back(highest);

		// left
		vector<Point> left;
		nextPt = findNextCvxHullPt(lowestIt, true);
		while (nextPt->second != highest.second) {
			left.push_back(*nextPt);
			nextPt = findNextCvxHullPt(nextPt, true);
		}

		// combine
		for (auto it = left.rbegin(); it != left.rend(); it++) {
			cvxHull.push_back(*it);
		}

		return cvxHull;
	}

	void runIntersect() {
		disp = CImgDisplay(img, "Intersect");

		while (!disp.is_closed()) {
			disp.wait();
			if (disp.button()) {
				img.fill(0);
				auto points = genNPoints(15);

				// p1p2 p3p4
				auto &p1 = points[1], &p2 = points[2], &p3 = points[3], &p4 = points[4];
				drawLine(p1, p2);
				drawLine(p3, p4);

				string txt;
				if (intersect(p1, p2, p3, p4)) txt = "intersect";
				else txt = "not intersect";
				img.draw_text(10, 10, txt.c_str(), 15, white, 1.0f, 20);
				disp.display(img);
			}
		}
	}

	void runSweepingSegment() {
		disp = CImgDisplay(img, "Click a point");

		while (!disp.is_closed()) {
			disp.wait();
			if (disp.button() & 1) {
				img.fill(0);
				auto pts = genNPoints(15);
				auto lines = linkMLines(pts, 5);

				disp.display(img);

				// positive: open  neg: close
				vector<pair<Point, int>> linePts;

				for (int i = 0; i < lines.size(); i++) {
					linePts.push_back({ lines[i].first, i });
					linePts.push_back({ lines[i].second, -i });
				}

				sort(linePts.begin(), linePts.end(), [](auto x, auto y) {return x.first.first < y.first.first; });
				int sweep = 0;
				while (!disp.is_closed()) {
					disp.wait();
					if (disp.button() & 2) {
						while (sweep < linePts.size()) {
							img.fill(0);
							drawPoints(pts);
							drawLines(lines);

							// draw sweep line
							int sweepX = linePts[sweep].first.first;
							img.draw_line(sweepX, 0, 0, sweepX, height, 0, white);
							disp.display(img);
							sweep++;

							Sleep(500);
						}
						break;
					}
					else if (disp.button() & 1) { break; }
				}

				disp.display(img);

			}
		}
	}

	vector<Line> linkMLines(const vector<Point> &pts, const int M) {
		set<pair<int, int>> s;
		vector<Line> lines;
		for (int i = 0; i < M; i++) {
			int pt1 = -1, pt2 = -1;
			while (pt1 == pt2) pt1 = mt() % pts.size(), pt2 = mt() % pts.size();
			if (s.find({ pt1,pt2 }) == s.end() && s.find({ pt2,pt1 }) == s.end()) {
				s.insert({ pt1, pt2 });
				if (pts[pt1].first < pts[pt2].first) lines.push_back({ pts[pt1], pts[pt2] });
				else lines.push_back({ pts[pt2],pts[pt1] });
			}
		}

		// draw lines
		for (auto l : lines)
			drawLine(l.first, l.second);

		return lines;
	}

	void drawPoint(int x, int y) {
		img.draw_rectangle(x, y, x + 1, y + 1, white);
	}

	void drawPoints(const vector<Point> &points) {
		for (auto p : points)
			drawPoint(p.first, p.second);
	}

	void drawLine(int x1, int y1, int x2, int y2) {
		img.draw_line(x1, y1, 0, x2, y2, 0, white);
	}

	void drawLine(pair<int, int> p1, pair<int, int> p2) {
		drawLine(p1.first, p1.second, p2.first, p2.second);
	}

	void drawLines(const vector<Line> &lines) {
		for (auto l : lines)
			drawLine(l.first, l.second);
	}

	/*
	Generate N points
	*/
	vector<Point> genNPoints(int n) {
		vector<pair<int, int>> ret;

		img.fill(0); // clear image
		while (n--) ret.push_back({ mt() % width, mt() % height });  // generate points
		drawPoints(ret); // draw points

		return ret;
	}

	/*
	1 : clockwise
	0 : same dir
	-1 : counter clockwise
	01 02
	*/
	int clockWise(int x1, int y1, int x2, int y2) {
		int v = x1 * y2 - x2 * y1;
		if (v > 0) return 1;
		else if (v < 0) return -1;
		else return 0;
	}

	/*
	01, 02
	1 : clockwise
	0 : same dir
	-1 : counter clockwise
	*/
	int clockWise(int x0, int y0, int x1, int y1, int x2, int y2) {
		return clockWise(x1 - x0, y1 - y0, x2 - x0, y2 - y0);
	}

	/*
	p1p2 p1p3
	1 : clockwise
	0 : same dir
	-1 : counter clockwise
	*/
	int clockWise(pair<int, int> p1, pair<int, int> p2, pair<int, int> p3) {
		return clockWise(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
	}

	/*
	1: left
	0:
	-1: right
	*/
	int consectiveClockWise(int x0, int y0, int x1, int y1, int x2, int y2) {
		return clockWise(x0, y0, x1, y1, x2, y2);
	}

	bool onSegment(Point x, Point p1, Point p2) {
		return std::min(p1.first, p2.first) <= x.first && x.first <= std::max(p1.first, p2.first)
			&& std::min(p1.second, p2.second) <= x.second && x.second <= std::max(p1.second, p2.second);
	}

	double distance(const Point &a, const Point &b) {
		return std::sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second)*(a.second - b.second));
	}
	/*
	p1p2 and p3p4 intersect->	p1p2 cross the line (not segment) of p3p4, and p3p4 cross the line of p1p2
	-> (vector p1p3 to p1p4) and (vector p2p3 to p2p4) are opposite turn, ie, one is clockwise and the other is counter-clockwise.
	same to p3p4.
	*/
	bool intersect(Point p1, Point p2, Point p3, Point p4) {
		int d1 = clockWise(p1, p3, p4),
			d2 = clockWise(p2, p3, p4),
			d3 = clockWise(p3, p1, p2),
			d4 = clockWise(p4, p1, p2);
		if (d1 * d2 < 0 && d3 * d4 < 0) return true;
		else if ((!d1&& onSegment(p1, p3, p4))
			|| (!d2 && onSegment(p2, p3, p4))
			|| (!d3 && onSegment(p3, p1, p2))
			|| (!d4 && onSegment(p4, p1, p2)))
			return true;
		else return false;
	}
};

int main() {
	ComputationalGeo c;
	c.run();
	return 0;
}
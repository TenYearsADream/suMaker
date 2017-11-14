#pragma once

//Geometric classes for suImage
namespace SU {

	// **********************
	// *                    *
	// *  Pixel Data Types  *
	// *                    *
	// **********************

	// It is very useful to keep pixel types separate to improve
	// readability and to make it easier to make changes.

	typedef unsigned char  GRAY8;      // 1-byte
	typedef unsigned short DEPTH16;    // 2-bytes
	typedef unsigned int   RGBA32u;    // 4-bytes  (Unsigned)
	typedef int            RGBA32s;    // 4-bytes  (Signed)

	// *************
	// *           *
	// *  Color    *
	// *           *
	// *************

	// [R G B alpha] = a unsigned int color 
	class Color 
	{
	public:
		Color(unsigned char R, unsigned char G,
			unsigned char B, unsigned char A) {
			rgba_ = R << 24 | G << 16 | B << 8 | A;
		}
		RGBA32u color() { return rgba_; }

		unsigned char getR() { return rgba_ & 0xFF000000 >> 24; }
		unsigned char getG() { return rgba_ & 0x00FF0000 >> 16; }
		unsigned char getB() { return rgba_ & 0x0000FF00 >> 8; }
		unsigned char getA() { return rgba_ & 0x000000FF; }

	private:
		RGBA32u rgba_;
	};

	// *************
	// *           *
	// *  min/max  *
	// *           *
	// *************
	template <class T> const T& suMin(const T& a, const T& b){
		return (a < b) ? a : b;
	}

	template <class T> const T& suMax(const T& a, const T& b){
		return (a > b) ? a : b;
	}

	// *************
	// *           *
	// *  Point    *
	// *           *
	// *************
	class suPoint
	{
	public:
		suPoint() : x_(0), y_(0) {};
		suPoint(int x, int y) : x_(x), y_(y) {}

		int x() const { return x_; }
		int y() const { return y_; }

		bool operator == (const suPoint& p) const{
			return x() == p.x() && y() == p.y();
		}

		suPoint& operator += (const suPoint& p) {
			x_ += p.x(); y_ += p.y();
			return *this;
		}

		// Default copy constructor and assignment operators ok
	private:
		int x_, y_;

	};

	// ************
	// *          *
	// *  suRect  *
	// *          *
	// ************
	//
	// A rect defined by image coordinate: upper left and lower right

	class suRect
	{
	public:
		suRect() : width_(0), height_(0) {}
		suRect(suPoint ul, unsigned int width, unsigned int height) : ul_(ul), width_(width), height_(height) {}
		suRect(suPoint ul, suPoint lr) : ul_(ul), width_(lr.x() - ul.x()), height_(lr.y() - ul.y()){}
		suRect(int x0, int y0, unsigned int width, unsigned int height) : ul_(suPoint(x0, y0)), width_(width), height_(height) {}

		const suPoint& ul() const { return ul_; }
		suPoint        lr() const { return suPoint(ul_.x() + width_, ul_.y() + height_); }
		// Returns upper-left and lower-right coordinates

		int  x0() const { return ul_.x(); }
		int  y0() const { return ul_.y(); }
		int  x1() const { return lr().x(); }
		int  y1() const { return lr().y(); }

		unsigned int  width() const { return width_; }
		unsigned int  height() const { return height_; }

		bool  isNull() const { return width_ == 0 || height_ == 0; }
		// Returns true if the rectangle is null

		bool  operator == (const suRect& r) const { return ul_ == r.ul() && width_ == r.width() && height_ == r.height(); }
		bool  operator != (const suRect& r) const{
			return !operator== (r);
		}

		bool  within(const suPoint& p) const { // Returns true if the point is within the rectangle
			return (p.x() >= ul_.x()) && (p.y() >= ul_.y()) && 
				   (p.x() <= lr().x()) && (p.y() < lr().y());
		}
		
		suRect intersect(const suRect& r) const { // Returns the intersection of two rectangles
			const suPoint&  ul1 = ul();
			const suPoint&  ul2 = r.ul();
			int x = suMax(ul1.x(), ul2.x());
			int y = suMax(ul1.y(), ul2.y());
			int w = suMin(ul1.x() + width(), ul2.x() + r.width()) - x;
			int h = suMin(ul1.y() + height(), ul2.y() + r.height()) - y;

			if (w < 0 || h < 0) return suRect();

			return suRect(x, y, w, h);
		}

		void expand(int x, int y) {// expand or shrink
			if (!isNull()) {
				ul_ += suPoint(-x, -y);
				width_ += 2 * x;
				height_ += 2 * y;
			}
		}
		

		// Default copy constructor and assignment operators ok.
	private:
		suPoint ul_;          // Upper-left hand coordinates
		unsigned int width_;  // Image width
		unsigned int height_; // Image height
	};



}


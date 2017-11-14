#pragma once
/* This file consists of header only image classes, which provide simple operation for 2D images.
 *
 */
#include <atomic>
#include "geometry.h"

namespace SU {
	class  Object {
	public:
		/// Default constructor
		Object() { }

		/// Copy constructor
		Object(const Object &) : refCount_(0) {}

		/// Return the current reference count
		int getRefCount() const { return refCount_; };

		/// Increase the object's reference count by one
		void incRef() const { ++refCount_; }

		/** \brief Decrease the reference count of
		* the object and possibly deallocate it.
		*
		* The object will automatically be deallocated once
		* the reference count reaches zero.
		*/
		void decRef(bool dealloc = true) const {
			--refCount_;
			if (refCount_ == 0 && dealloc)
				delete this;
			else if (refCount_ < 0)
				throw std::runtime_error("Internal error: reference count < 0!");
		}
	protected:
		/** \brief Virtual protected deconstructor.
		* (Will only be called by \ref ref)
		*/
		virtual ~Object() { }
	private:
		mutable std::atomic<int> refCount_{ 0 };
	};

	// ****************
	// *              *
	// *	Ref       *
	// *              *
	// ****************
	// \brief a refence template, which wraps a pointer to a class 
	// that mube be derived from Object
	
	template<typename T>
	class Ref
	{
	public:
		// Create a nullptr refernce
		Ref() : ptr_(nullptr) {}

		// Construct a reference from a pointer
		Ref(T *ptr) : ptr_(ptr) {
			if(ptr_)  ((Object *)ptr_)->incRef();
		}

		// Copy constructor
		Ref(const Ref &r) : ptr_(r.ptr_) {
			if (ptr_) ((Object *)ptr_)->incRef();
		}

		// Move constructor
		Ref(Ref &&r) : ptr_(r.ptr_) {
			r.ptr_ = nullptr;
		}

		// Destroy this reference
		~Ref() {
			if (ptr_) ((Object*)ptr_)->decRef(); 
		}

		// Move another reference into the current one
		Ref& operator=(Ref&& r) {
			if (*this == r)
				return *this;
			if (ptr_)
				((Object *)ptr_)->decRef();
			ptr_ = r.ptr_;
			r.ptr_ = nullptr;
			return *this;
		}

		// Overwrite this reference with another reference
		Ref& operator=(const Ref& r) {
			if (ptr_ == r.ptr_)
				return *this;
			if (ptr_)
				((Object *)ptr_)->decRef();
			ptr_ = r.ptr_;
			if (ptr_)
				((Object *)ptr_)->incRef();
			return *this;
		}

		/// Overwrite this reference with a pointer to another object
		Ref& operator=(T *ptr) {
			if (ptr_ == ptr)
				return *this;
			if (ptr_)
				((Object *)ptr_)->decRef();
			ptr_ = ptr;
			if (ptr_)
				((Object *)ptr_)->incRef();
			return *this;
		}

		/// Compare this reference with another reference
		bool operator==(const Ref &r) const { return ptr_ == r.ptr_; }

		/// Compare this reference with another reference
		bool operator!=(const Ref &r) const { return ptr_ != r.ptr_; }

		/// Compare this reference with a pointer
		bool operator==(const T* ptr) const { return ptr_ == ptr; }

		/// Compare this reference with a pointer
		bool operator!=(const T* ptr) const { return ptr_ != ptr; }

		/// Access the object referenced by this reference
		T* operator->() { return ptr_; }

		/// Access the object referenced by this reference
		const T* operator->() const { return ptr_; }

		/// Return a C++ reference to the referenced object
		T& operator*() { return *ptr_; }

		/// Return a const C++ reference to the referenced object
		const T& operator*() const { return *ptr_; }

		/// Return a pointer to the referenced object
		operator T* () { return ptr_; }

		/// Return a const pointer to the referenced object
		T* get() { return ptr_; }

		/// Return a pointer to the referenced object
		const T* get() const { return ptr_; }

		/// Check if the object is defined
		operator bool() const { return ptr_ != nullptr; }
	private:
		T * ptr_;
		
	};


	// ************************
	// *                      *
	// *	suAllocBase_      *
	// *                      *
	// ************************
	// A base class for different allocater
	template<typename T>
	class suAllocBase_ : public Object
	{
	public:
		suAllocBase_(unsigned int n) : pData_(nullptr), size_(n) {}

		virtual ~suAllocBase_() {}
		// Derived classes will deallocate memory.

		operator T* () { return pData_; }
		operator const T* () const { return pData_; }
		// Conversion to pointer of allocated memory type

		unsigned int size() const { return size_; }  // Number of elements

		
		T* data() { return pData_; }

		const T& operator[] (unsigned int index) const {
			if (index >= size())
				throw std::range_error("Index out of range");
			return *(pData_ + index);
		}

		T&  operator[] (unsigned int index) {
			if (index >= size())
				throw std::range_error("Index out of range");
			return *(pData_ + index);
		}

	protected:
		virtual void allocate() = 0;
		virtual void deallocate() = 0;


		T*       pData_;  // Address pointer in memory(todo: deal with GPU)
		unsigned int size_;   // Number of elements allocated
	};

	// ************************
	// *                      *
	// *	suAllocatorCPU    *
	// *                      *
	// ************************
	// 
	template<class T>
	class suAllocatorCPU : public suAllocBase_<T>
	{
	public:
		suAllocatorCPU(unsigned int n) : suAllocBase_(n) {
			allocate();
		}
		~suAllocatorCPU() {
			deallocate();
		}

		virtual void allocate() {
			//Simply use new
			pData_ = new T[size_];
		}

		virtual void deallocate() {
			if (pData_ != nullptr) {
				delete pData_;
				pData_ = nullptr;
			}
		}


	};

	
	
	// ************************
	// *                      *
	// *	suImage           *
	// *                      *
	// ************************


	template<class T, class A = suAllocatorCPU<T> >
	class suImage
	{
	public:
		suImage():width_(0),height_(0), s_(nullptr) {}
		suImage(int width, int height) : width_(width), height_(height) {
			s_ = new A(width_ * height_);
			rect_ = suRect(0, 0, width_, height_);
		}
		~suImage() { cleanup(); }

		int width() { return width_; }
		int height() { return height_; }

		T* data() { return pixels_; }
		A& storage() { return *s_; }

		//reallocate 
		void resize(unsigned int width, unsigned int height) {
			cleanup();
			width_ = width;
			height_ = height;
			s_ = new A(width_ * height_);
			rect_ = suRect(0, 0, width_, height_);
		}

		void setPixel(int x, int y, T pixel){
			(*s_)[width_ * y + x] = pixel;
		}

		const T& getPixel(int x, int y) const {
			return (*s_)[width_ * y + x];
		}
		void set(T pixel, suRect& rect = suRect() ) {
			if (rect.height() == 0 && rect.width() == 0)
				rect = rect_;
			suRect fill_rect = rect_.intersect(rect);
			A& pMem = *s_;

			int yStart = fill_rect.ul().y();
			int xStart = fill_rect.ul().x();
			for (unsigned int j = 0; j < fill_rect.height(); j++) {
				for (unsigned int i = 0; i < fill_rect.width(); i++) {
					pMem[(yStart + j) * width_ + xStart + i] = pixel;
				}
			}
		}

		//suImage& operator= (const suImage& im)

	private:
		void cleanup() {
			// if(s_ != nullptr)  delete s_;
			// Ref<A> is used, so the pointer will be released automatically
		}

		int width_;
		int height_;

		suRect rect_;    //store the image rect

		
		Ref<A> s_;
	};


}
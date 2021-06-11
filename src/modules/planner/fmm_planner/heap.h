#pragma once

#include <vector>
#include <algorithm>




template<typename _Comp>
class Heap
{
protected:
	std::vector<unsigned long> c;	// underlying container
  _Comp comp;				        // comparator functor,comp(a,b)当a<b返回true
    unsigned long *m;				// mapping from position to index in the container

public:
	explicit Heap(const _Comp& _Comparator, const unsigned long max_size) :
		c(), comp(_Comparator), m(new unsigned long[max_size])
	{	// construct with specified comparator
	}

	~Heap()
	{
		delete[] m;
	}
	
	bool empty() const
	{	// test if the heap is empty
		return c.empty();
	}

    unsigned long size() const
	{	// return length of queue
		return c.size();
	}

    unsigned long top() const
	{
		return c.front();
	}

	void push(const unsigned long item)
	{
		// append to the end of the container
		c.push_back(item);
		// save mapping
		m[item] = size() - 1;
		// rebuild heap property from bottom to top
		sift_up(size() - 1);
	}

	void pop()
	{
		m[c.back()] = 0;
		// swap root with the last element and remove it from the container
		std::swap(c.front(), c.back());
		c.pop_back();
		// rebuild the heap property
		sift_down(0);
	}

	void update(const unsigned long item)
	{
    unsigned long start = m[item];
    sift_up(start);//如果当前节点的值比父节点小，向上移动
    sift_down(start);//确认交换过来的节点是否需要向下移动
	}

	//void print()
	//{
	//	cout << "container" << endl << "[";
	//	for (unsigned long i : c)
	//		cout << i << ", ";
	//	cout << "]" << endl;

	//	//std::cout << "mappings" << std::endl;
	//	//for (std::map<_ItemType, unsigned long>::iterator i=m.begin(); i != m.end(); i++)
	//	//	std::cout << i->first << ":" << i->second << ", ";
	//	//std::cout << std::endl;
	//}

private:
	inline unsigned long parent(const unsigned long i) const
	{
    return (i - 1) >> 1;//获取父节点编号
	}

	inline unsigned long left(const unsigned long i) const
	{
    return (i << 1) + 1;//获取左子节点编号
	}

	inline unsigned long right(const unsigned long i) const
	{
    return (i << 1) + 2;//获取右子节点编号
	}
  // shiftDown(): 如果一个节点比它的子节点小（最大堆）或者大（最小堆），那么需要将它向下移动。这个操作也称作“堆化（heapify）”。
	void sift_down(const unsigned long start)
	{
        unsigned long s = size();
        unsigned long i = start;

		while (true)
		{
      unsigned long min = i;
      // 如果左子节点的值小于当前节点，最小点编号移交左子节点
			if (left(i) < s && comp(c[left(i)], c[min]))
				min = left(i);
      // 如果右子节点的值小于左子节点，最小点编号移交右子节点
			if (right(i) < s && comp(c[right(i)], c[min]))
				min = right(i);
      // 如果最小节点就是当前节点，中断循环
			if (min == i)
				break;
      //将当前节点与最小节点进行互换
			std::swap(m[c[i]], m[c[min]]);
			std::swap(c[i], c[min]);
			i = min;
		}
	}
  //shiftUp(): 如果一个节点比它的父节点大（最大堆）或者小（最小堆），那么需要将它同父节点交换位置。这样是这个节点在数组的位置上升。
	void sift_up(const unsigned long start)
	{
        unsigned long i = start;
		while (true)
		{
      //如果点已经位于最小堆顶点
      if (i == 0)
				break;
      //找到父节点的编号
      unsigned long p = parent(i);
      // 如果父节点的值大于子节点，就需要进行交换，因此是最小堆
			if (comp(c[i], c[p]))
			{
        std::swap(m[c[i]], m[c[p]]);
				std::swap(c[i], c[p]);
				i = p;
			}
      // 如果点已经位于堆中的合适位置
			else
      {
        break;
      }
		}
	}
};


//int main()
//{
//	float* tau = new float[6]{ 1, 2, 3, 4, 5, 6 };
//	auto heap_comp = [&tau](const long e1, const long e2) { return tau[e1] < tau[e2]; };
//	Heap<long, decltype(heap_comp)> h(heap_comp);
//
//	h.push(0);
//	h.push(1);
//	h.push(2);
//
//	std::cout << "brefore update" << std::endl;
//	h.print();
//
//	tau[1] = 0;
//	h.update(1);
//	
//	std::cout << std::endl;
//	std::cout << "after update" << std::endl;
//	h.print();
//
//
//	std::cout << "size: " << h.size() << std::endl;
//	while (!h.empty())
//	{
//		std::cout << h.top() << ":" << tau[h.top()] << ", ";
//		h.pop();
//	}
//	std::cout << std::endl;
//
//	return 0;
//}

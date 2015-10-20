template <class Container, class UnaryPredicate>
inline void filter(Container& container, UnaryPredicate pred) {
	auto first = container.begin(), last = container.end(), newEnd = first;
	while (first!=last) {
		if (pred(*first)) {
			*newEnd = std::move(*first);
			++newEnd;
		}
		++first;
	}
	container.resize(std::distance(container.begin(), newEnd));
}

template <class Container, class UnaryPredicate>
inline void remove(Container& container, UnaryPredicate pred) {
	auto newEnd = std::remove_if(container.begin(), container.end(), pred);
	container.resize(std::distance(container.begin(), newEnd));
}

template <class Container, class UnaryPredicate>
inline void filterIdx(Container& container, UnaryPredicate pred) {
	auto first = container.begin(), last = container.end(), newEnd = first;
	while (first != last) {
		if (bool(pred(std::distance(container.begin(), first)))) {
			*newEnd = *first;
			++newEnd;
		}
		++first;
	}
	container.resize(std::distance(container.begin(), newEnd));
}

template <class Container, class UnaryPredicate>
inline void removeIdx(Container& container, UnaryPredicate pred) {
	typename Container::iterator begin  = container.begin();
	typename Container::iterator newEnd = begin;
	while (begin != container.end()) {
		if (!bool(pred(std::distance(container.begin(), begin)))) {
			*newEnd = *begin;
			++newEnd;
		}
		++begin;
	}
	container.resize(std::distance(container.begin(), newEnd));
}

template <class Container, class IndexSet>
inline Container slice(const Container& container, const IndexSet& indices) {
	Container result(indices.size());
	typename IndexSet::value_type lastIndex = 0;
	auto iterIn = container.begin(), iterOut = result.begin();
	for (const auto& idx : indices) {
		std::advance(iterIn, idx-lastIndex);
		*iterOut = *iterIn;
		++iterOut;
		lastIndex = idx;
	}
	return result;
}

template <class Container, class IndexSet>
inline void slice(Container& container, const IndexSet& indices) {
	Container result = slice<Container,IndexSet>(container, indices);
	container = result;
}

template <class Container, class IndexSet>
inline Container crop(const Container& container, const IndexSet& indices) {
	typedef typename IndexSet::value_type IdxType;
	IndexSet neg(container.size());
	std::iota(neg.begin(), neg.end(), 0);
	removeIf(neg, [&] (IdxType idx) { return std::binary_search(indices.begin(), indices.end(), idx); });
	return slice(container, neg);
}

template <class Container, class IndexSet>
inline void crop(Container& container, const IndexSet& indices) {
	Container result = crop<Container,IndexSet>(container, indices);
	container = result;
}

template <class Container>
inline Container setUnion(const Container& container0, const Container& container1) {
	Container result(container0.size()+container1.size());
	auto newEnd = std::set_union(container0.begin(), container0.end(), container1.begin(), container1.end(), result.begin());
	result.resize(std::distance(result.begin(), newEnd));
	return result;
}

template <class Container>
inline Container setIntersection(const Container& container0, const Container& container1) {
	Container result(container0.size());
	auto newEnd = std::set_intersection(container0.begin(), container0.end(), container1.begin(), container1.end(), result.begin());
	result.resize(std::distance(result.begin(), newEnd));
	return result;
}

template <class Container>
inline Container setDifference(const Container& container0, const Container& container1) {
	Container result(container0.size());
	auto newEnd = std::set_difference(container0.begin(), container0.end(), container1.begin(), container1.end(), result.begin());
	result.resize(std::distance(result.begin(), newEnd));
	return result;
}

template <class Container>
inline Container setSymmetricDifference(const Container& container0, const Container& container1) {
	Container result(container0.size()+container1.size());
	auto newEnd = std::set_symmetric_difference(container0.begin(), container0.end(), container1.begin(), container1.end(), result.begin());
	result.resize(std::distance(result.begin(), newEnd));
	return result;
}

template <class Container>
inline void append(Container& container0, const Container& container1) {
	container0.insert(container0.end(), container1.begin(), container1.end());
}

template <class Container>
inline Container append(const Container& container0, const Container& container1) {
	Container result(container0.begin(), container0.end());
	append(result, container1);
	return result;
}

template <class Container>
inline void uniqueSubset(Container& container) {
	std::sort(container.begin(), container.end());
	auto newEnd = std::unique(container.begin(), container.end());
	container.resize(std::distance(container.begin(), newEnd));
}

template <class Container>
inline void pairwise(const Container& container, std::function<void (const typename Container::value_type&, const typename Container::value_type&)> func) {
	for (auto it0 = container.begin(); it0 != container.end(); ++it0) {
		for (auto it1 = container.begin(); it1 != container.end(); ++it1) {
			if (it0 != it1) func(*it0, *it1);
		}
	}
}

template <class Container>
inline void pairwise(Container container, std::function<void (typename Container::value_type&, typename Container::value_type&)> func) {
	for (auto it0 = container.begin(); it0 != container.end(); ++it0) {
		for (auto it1 = container.begin(); it1 != container.end(); ++it1) {
			if (it0 != it1) func(*it0, *it1);
		}
	}
}

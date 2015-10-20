#ifndef ALGOSETS_H_
#define ALGOSETS_H_

#include <algorithm>
#include <functional>

namespace Algorithm {

template <class Container, class UnaryPredicate>
void filter(Container& container, UnaryPredicate pred);

template <class Container, class UnaryPredicate>
void remove(Container& container, UnaryPredicate pred);

template <class Container, class UnaryPredicate>
void filterIdx(Container& container, UnaryPredicate pred);

template <class Container, class UnaryPredicate>
void removeIdx(Container& container, UnaryPredicate pred);

template <class Container, class IndexSet>
Container slice(const Container& container, const IndexSet& indices);

template <class Container, class IndexSet>
void slice(Container& container, const IndexSet& indices);

template <class Container, class IndexSet>
Container crop(const Container& container, const IndexSet& indices);

template <class Container, class IndexSet>
void crop(Container& container, const IndexSet& indices);

template <class Container>
Container setUnion(const Container& container0, const Container& container1);

template <class Container>
Container setIntersection(const Container& container0, const Container& container1);

template <class Container>
Container setDifference(const Container& container0, const Container& container1);

template <class Container>
Container setSymmetricDifference(const Container& container0, const Container& container1);

template <class Container>
void append(Container& container0, const Container& container1);

template <class Container>
Container append(const Container& container0, const Container& container1);

template <class Container>
void uniqueSubset(Container& container);

template <class Container>
void pairwise(const Container& container, std::function<void (const typename Container::value_type&, const typename Container::value_type&)> func);

template <class Container>
void pairwise(Container container, std::function<void (typename Container::value_type&, typename Container::value_type&)> func);

#include "Sets.inl"

} // Algorithm

#endif /* ALGOSETS_H_ */

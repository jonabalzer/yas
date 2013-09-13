#include "Octree.h"

namespace PoissonRec {

template<class NodeData,class Real> const int OctNode<NodeData,Real>::DepthShift=5;
template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift=19;
template<class NodeData,class Real> const int OctNode<NodeData,Real>::DepthMask=(1<<DepthShift)-1;
template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetMask=(1<<OffsetShift)-1;
template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift1=DepthShift;
template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift2=OffsetShift1+OffsetShift;
template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift3=OffsetShift2+OffsetShift;



}

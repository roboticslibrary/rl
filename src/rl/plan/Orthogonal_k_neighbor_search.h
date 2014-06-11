// Copyright (c) 2002,2011 Utrecht University (The Netherlands).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/branches/next/Spatial_searching/include/CGAL/Orthogonal_k_neighbor_search.h $
// $Id: Orthogonal_k_neighbor_search.h 67117 2012-01-13 18:14:48Z lrineau $
// 
//
// Author(s)     : Gael Guennebaud (gael.guennebaud@inria.fr), Hans Tangelder (<hanst@cs.uu.nl>)
//                 Markus Rickert

#ifndef _RL_PLAN_ORTHOGONAL_K_NEIGHBOR_SEARCH_H_
#define _RL_PLAN_ORTHOGONAL_K_NEIGHBOR_SEARCH_H_

#include <CGAL/internal/K_neighbor_search.h>

namespace rl { namespace plan {

template <class SearchTraits, 
#if (CGAL_VERSION_NR > 1030801000)
          class Distance= typename ::CGAL::internal::Spatial_searching_default_distance<SearchTraits>::type,
#else
          class Distance= ::CGAL::Euclidean_distance<SearchTraits>,
#endif
          class Splitter= ::CGAL::Sliding_midpoint<SearchTraits> ,
          class Tree= ::CGAL::Kd_tree<SearchTraits, Splitter, ::CGAL::Tag_true> >
class Orthogonal_k_neighbor_search: public ::CGAL::internal::K_neighbor_search<SearchTraits,Distance,Splitter,Tree> {
  typedef  ::CGAL::internal::K_neighbor_search<SearchTraits,Distance,Splitter,Tree> Base;

  typename SearchTraits::Cartesian_const_iterator_d query_object_it;
  
public:
  typedef typename Base::FT FT;

#if (CGAL_VERSION_NR > 1030901000)
  Orthogonal_k_neighbor_search(const Tree& tree, const typename Base::Query_item& q,
#else
  Orthogonal_k_neighbor_search(Tree& tree, const typename Base::Query_item& q,
#endif
                               unsigned int k=1, FT Eps=FT(0.0), bool Search_nearest=true, const Distance& d=Distance(),bool sorted=true)
    : Base(q,k,Eps,Search_nearest,d) 
  {
    if (tree.empty()) return;

    FT distance_to_root;
    if (this->search_nearest) 
      distance_to_root = this->distance_instance.min_distance_to_rectangle(q, tree.bounding_box());
    else 
      distance_to_root = this->distance_instance.max_distance_to_rectangle(q, tree.bounding_box());


#if (CGAL_VERSION_NR > 1030801000)
    typename SearchTraits::Construct_cartesian_const_iterator_d construct_it=tree.traits().construct_cartesian_const_iterator_d_object();
#else
    typename SearchTraits::Construct_cartesian_const_iterator_d construct_it;
#endif
    query_object_it = construct_it(this->query_object);
      
    compute_neighbors_orthogonally(tree.root(), distance_to_root);      
      
    if (sorted) this->queue.sort();
  }
private:

#if (CGAL_VERSION_NR > 1030901000)
  void compute_neighbors_orthogonally(typename Base::Node_const_handle N, FT rd)
#else
  void compute_neighbors_orthogonally(typename Base::Node_handle N, FT rd)
#endif
  {
    if (!(N->is_leaf())) 
    {
      this->number_of_internal_nodes_visited++;
      int new_cut_dim=N->cutting_dimension();
      FT new_rd;
      FT low_off = this->distance_instance.min_distance_to_rectangle(
        *(query_object_it + new_cut_dim),
        N->low_value(),
        N->cutting_value(),
        new_cut_dim
      );
      FT high_off = this->distance_instance.min_distance_to_rectangle(
        *(query_object_it + new_cut_dim),
        N->cutting_value(),
        N->high_value(),
        new_cut_dim
      );
      if ( ((low_off < high_off) && (this->search_nearest)) ||
           ((low_off >= high_off) && (!this->search_nearest)) ) 
      {
        compute_neighbors_orthogonally(N->lower(),rd);
        if (this->search_nearest) {
        	new_rd = this->distance_instance.new_distance(rd,low_off,high_off,new_cut_dim);
        }
        else {	
        	new_rd = this->distance_instance.new_distance(rd,high_off,low_off,new_cut_dim);
        }
        if (this->branch(new_rd)) 
          compute_neighbors_orthogonally(N->upper(), new_rd);                               
      }
      else // compute new distance
      {
        compute_neighbors_orthogonally(N->upper(),rd); 
        if (this->search_nearest) {
        	new_rd = this->distance_instance. new_distance(rd,high_off,low_off,new_cut_dim);
        }
        else  {       
        	new_rd = this->distance_instance. new_distance(rd,low_off,high_off,new_cut_dim);
        }  
        if (this->branch(new_rd)) 
          compute_neighbors_orthogonally(N->lower(), new_rd);       
      }
    }
    else
    {
      // n is a leaf
      this->number_of_leaf_nodes_visited++;
      if (N->size() > 0)
      {
        for (typename Base::Point_d_iterator it=N->begin(); it != N->end(); it++) 
        {
          this->number_of_items_visited++;
          FT distance_to_query_object=
            this->distance_instance.transformed_distance(this->query_object,**it);
          this->queue.insert(::std::make_pair(*it,distance_to_query_object));
        }
      }
    }
  }    
  
}; // class 

} } // namespace

#endif  // _RL_PLAN_ORTHOGONAL_K_NEIGHBOR_SEARCH_H_

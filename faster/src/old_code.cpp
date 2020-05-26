
// if (checkFeasAndFillND(result_, n, d) == false)  // This may be true or false, depending on the case
// {
//   return false;
// }
// else
// {
//   return true;
// }

/*
  return false;

  switch (status)
  {
    case GOAL_REACHED:
      recoverPath(current_ptr);                       // saved in result_
      if (checkFeasAndFillND(result, n, d) == false)  // This should always be true
      {
        return false;
      }
      if (visual_)
      {
        // plotExpandedNodesAndResult(expanded_nodes_, current_ptr);
      }
      return true;

    case RUNTIME_REACHED:
  }

  if (closest_result_so_far_ptr_ == NULL || (closest_result_so_far_ptr_->index == 2))
  {
    std::cout << " and couldn't find any solution" << std::endl;
    if (visual_)
    {
      plotExpandedNodesAndResult(expanded_nodes_, closest_result_so_far_ptr_);
    }
    return false;
  }
  else
  {
    // Fill the until we arrive to N_-2, with the same qi
    // Note that, by doing this, it's not guaranteed feasibility wrt a dynamic obstacle
    // and hence the need of the function checkFeasAndFillND()
    for (int j = (closest_result_so_far_ptr_->index) + 1; j <= N_ - 2; j++)
    {
      Node* node_ptr = new Node;
      node_ptr->qi = closest_result_so_far_ptr_->qi;
      node_ptr->index = j;
      std::cout << "Filled " << j << ", ";
      // << node_ptr->qi.transpose() << std::endl;
      node_ptr->previous = closest_result_so_far_ptr_;
      closest_result_so_far_ptr_ = node_ptr;
    }
    std::cout << std::endl;

    // std::cout << " best solution has dist=" << closest_dist_so_far_ << std::endl;

    std::cout << "calling recoverPath3" << std::endl;
    recoverPath(closest_result_so_far_ptr_);  // saved in result_
    std::cout << "called recoverPath3" << std::endl;

    if (visual_)
    {
      plotExpandedNodesAndResult(expanded_nodes_, closest_result_so_far_ptr_);
    }

    if (checkFeasAndFillND(result_, n, d) == false)  // This may be true or false, depending on the case
    {
      return false;
    }

    return true;
  }

  return false;*/

//////

// std::cout << " neighbor.qi =" << neighbor.qi.transpose() << std::endl;

// std::cout << "orig_ = " << orig_.transpose() << std::endl;
// std::cout << "voxel_size_ = " << voxel_size_ << std::endl;
/////////////////
// std::cout << "New point at " << ix << ", " << iy << ", " << iz << std::endl;
// std::cout << neighbor.qi.transpose() << std::endl;

////////////////////////////////
// auto ptr_to_node = (*ptr_to_voxel).second;
// auto node_tmp = (*ptr_to_node);
// already_exists_with_lower_cost = (node_tmp.g + bias_ * (node_tmp.h)) < (neighbor.g + bias_ * neighbor.h);
//////////////////////////////////////////////
// std::cout << "constraint_xU= " << constraint_xU << std::endl;
// std::cout << "constraint_xL= " << constraint_xL << std::endl;
// std::cout << "delta_x= " << delta_x << std::endl;
// std::cout << "_____________________________________________" << std::endl;
// std::cout << "_____________________________________________" << std::endl;
// std::cout << "_____________________________________________" << std::endl;
// std::cout << "The neighbors of " << current.qi.transpose() << " are " << std::endl;
//////////////////////////////////////////////
// std::cout << bold << red << "map_open_list_ contains " << reset << std::endl;
// for (auto tmp : map_open_list_)
// {
//   std::cout << red << (*(tmp.second)).qi.transpose() << reset << std::endl;
// }

// std::cout << bold << "openList_ contains " << reset << std::endl;
// for (auto tmp : openList_)
// {
//   std::cout << tmp.qi.transpose() << std::endl;
// }

// std::cout << bold << "openList_ contains " << reset << std::endl;
// for (auto tmp : expanded_nodes_)
// {
//   std::cout << tmp.qi.transpose() << std::endl;
// }

/////////////////////////////////////////////////
// if (already_exists_with_lower_cost == true)
// {
//   std::cout << "already_exists_with_lower_cost" << std::endl;

//   continue;
// }

// if (already_exist == false)
// {
//   std::cout << red << neighbor.qi.transpose() << " does NOT exist" << reset << std::endl;
// }

////////////////////////////////////////////////
// std::cout << "size openList= " << openList_.size() << std::endl;
// if (already_exist)
// {
// deleteFromOpenListtheOldOne
// auto it = std::find_if(openList_.begin(), openList_.end(), boost::bind(&Node::index, _1) == neighbor.index);
//   std::cout << "trying to erase from openList_" << (*((*ptr_to_voxel).second)).qi.transpose() << std::endl;

// Option 1
// openList_.erase((*ptr_to_voxel).second);

// Option 3
// openList_.remove((*ptr_to_voxel).second);

// and now set the direction in the map to Null:
//(*ptr_to_voxel).second = nullptr;
// I should not use more times this pointer. But have to leave it here because it needs to keep "existing" so
// that I don't expand again that same voxel

//   std::cout << "erased" << std::endl;
// }

// typedef std::set<Node, CompareCost> my_list;
// typedef std::set<Node, CompareCost>::iterator my_list_iterator;

// std::cout << "other" << std::endl;

// std::pair<my_list_iterator, bool> ret;
// std::cout << "inserting to openList_" << neighbor.qi.transpose() << std::endl;

// Option 1
// ret = openList_.insert(neighbor);
// if (!ret.second)
// {
//   std::cout << "no insertion!\n";
// }
// map_open_list_[Eigen::Vector3i(ix, iy, iz)] = ret.first;  // Keep an iterator to neighbor

// Option 3
// auto iterator_tmp = openList_.pushAndReturnIterator(neighbor);
// map_open_list_[Eigen::Vector3i(ix, iy, iz)] = iterator_tmp;  // Keep an iterator to neighbor

// std::cout << red << "inserting to map_open_list_" << neighbor.qi.transpose() << reset << std::endl;
// std::cout << "other_done" << std::endl;

/////////////////////////////////////////////////

/*          std::cout << "ix= " << ix << " is outside, max= " << matrixExpandedNodes_.size() << std::endl;
          std::cout << "iy= " << iy << " is outside, max= " << matrixExpandedNodes_[0].size() << std::endl;
          std::cout << "iz= " << iz << " is outside, max= " << matrixExpandedNodes_[0][0].size() << std::endl;
          std::cout << "voxel_size_= " << voxel_size_ << std::endl;
          std::cout << "orig_= " << orig_.transpose() << std::endl;
          std::cout << "neighbor.qi= " << neighbor.qi.transpose() << std::endl;*/

/*  vx_.clear();
  vy_.clear();
  vz_.clear();

  for (int i = 0; i < num_samples_x_; i++)
  {
    vx_.push_back(-v_max_.x() + i * ((2.0 * v_max_.x()) / (num_samples_x_ - 1)));
  }
  for (int i = 0; i < num_samples_y_; i++)
  {
    vy_.push_back(-v_max_.y() + i * ((2.0 * v_max_.y()) / (num_samples_y_ - 1)));
  }
  for (int i = 0; i < num_samples_z_; i++)
  {
    vz_.push_back(-v_max_.z() + i * ((2.0 * v_max_.z()) / (num_samples_z_ - 1)));
  }

  std::cout << "vx is: " << std::endl;
  for (auto vxi : vx_)
  {
    std::cout << "vxi= " << vxi << std::endl;
  }
*/

/*  vx_.clear();
  vy_.clear();
  vz_.clear();

  for (int i = 0; i < num_samples_x_; i++)
  {
    vx_.push_back(constraint_xL + i * ((constraint_xU - constraint_xL) / (num_samples_x_ - 1)));
  }
  for (int i = 0; i < num_samples_y_; i++)
  {
    vy_.push_back(constraint_yL + i * ((constraint_yU - constraint_yL) / (num_samples_y_ - 1)));
  }
  for (int i = 0; i < num_samples_z_; i++)
  {
    vz_.push_back(constraint_zL + i * ((constraint_zU - constraint_zL) / (num_samples_z_ - 1)));
  }*/

/*  std::cout << "vx is: " << std::endl;
  for (auto tmp_i : vx_)
  {
    std::cout << tmp_i << std::endl;
  }
*/
// std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;
//    if (constraint_zL <= vz_i <= constraint_zU)
//    {
// std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;

/*  std::cout << "Current= " << current.qi.transpose() << std::endl;

  std::cout << "Current index= " << current.index << std::endl;
  std::cout << "knots= " << knots_ << std::endl;
  std::cout << "tmp= " << tmp << std::endl;
  std::cout << "a_max_.x() * tmp= " << a_max_.x() * tmp << std::endl;
  std::cout << "v_iM1.x()= " << v_iM1.x() << std::endl;*/

/*  vx_.clear();
  vy_.clear();
  vz_.clear();

  vx_.push_back(v_max_(0));
  vx_.push_back(v_max_(0) / 2.0);
  vx_.push_back(0);
  vx_.push_back(-v_max_(0) / 2.0);
  vx_.push_back(-v_max_(0));

  vy_.push_back(v_max_(1));
  vy_.push_back(v_max_(1) / 2.0);
  vy_.push_back(0);
  vy_.push_back(-v_max_(1) / 2.0);
  vy_.push_back(-v_max_(1));

  vz_.push_back(v_max_(2));
  vz_.push_back(v_max_(2) / 2.0);
  vz_.push_back(0);
  vz_.push_back(-v_max_(2) / 2.0);
  vz_.push_back(-v_max_(2));*/

/*  Eigen::Matrix<double, 4, 1> tmp_x;
  tmp_x << v_max_.x(), -v_max_.x(), a_max_.x() * tmp + viM1.x(), -a_max_.x() * tmp + viM1.x();
  double constraint_xU = tmp_x.maxCoeff();  // upper bound
  double constraint_xL = tmp_x.minCoeff();  // lower bound

  Eigen::Matrix<double, 4, 1> tmp_y;
  tmp_y << v_max_.y(), -v_max_.y(), a_max_.y() * tmp + viM1.y(), -a_max_.y() * tmp + viM1.y();
  double constraint_yU = tmp_y.maxCoeff();  // upper bound
  double constraint_yL = tmp_y.minCoeff();  // lower bound

  Eigen::Matrix<double, 4, 1> tmp_z;
  tmp_z << v_max_.z(), -v_max_.z(), a_max_.z() * tmp + viM1.z(), -a_max_.z() * tmp + viM1.z();
  double constraint_zU = tmp_z.maxCoeff();  // upper bound
  double constraint_zL = tmp_z.minCoeff();  // lower bound*/

/*  std::cout << "constraint_xL= " << constraint_xL << std::endl;
  std::cout << "constraint_xU= " << constraint_xU << std::endl;
  std::cout << "constraint_yL= " << constraint_yL << std::endl;
  std::cout << "constraint_yU= " << constraint_yU << std::endl;
  std::cout << "constraint_zL= " << constraint_zL << std::endl;
  std::cout << "constraint_zU= " << constraint_zU << std::endl;*/

/*    std::cout << "======================" << std::endl;

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> novale = openList;

    while (!novale.empty())
    {
      std::cout << (novale.top().h + novale.top().g) << " ";
      novale.pop();
    }*/

/*      double tentative_g = current.g + weightEdge(current, neighbor);  // Cost to come + new edge
      std::cout << "TEST" << std::endl;
      std::cout << "tentative_g=" << tentative_g << std::endl;
      std::cout << "g(neighbor)=" << g(neighbor) << std::endl;
      std::cout << "neighor.index=" << neighbor.index << std::endl;

      if (tentative_g < g(neighbor))
      {
        neighbor.previous = &current;
        neighbor.g = tentative_g;
        neighbor.f = neighbor.g + h(neighbor);
        // if neighbor not in OPEN SET TODO!!
        openList.push(neighbor);  // the order is automatic (it's a prioriry queue)
      }*/

/*  std::cout << "tmp=" << tmp << std::endl;

  std::cout << "vx_ =" << std::endl;

  for (double vx_i : vx_)
  {
    std::cout << vx_i << ", ";
  }
  std::cout << std::endl;

  for (double vy_i : vy_)
  {
    std::cout << vy_i << ", ";
  }
  std::cout << std::endl;

  for (double vz_i : vz_)
  {
    std::cout << vz_i << ", ";
  }
  std::cout << std::endl;*/

/*  if (current.index == 0)
  {
    Node nodeq1 = current;
    nodeq1.index = 1;
    nodeq1.previous = &current;
    nodeq1.g = 0;
    nodeq1.f = nodeq1.g + h(nodeq1);  // f=g+h

    neighbors.push_back(nodeq1);
    return neighbors;
  }

  if (current.index == 1)
  {
    Node nodeq2 = current;
    nodeq2.index = 1;
    nodeq2.previous = &current;
    nodeq2.g = current.previous->g + weightEdge(current, nodeq2);
    nodeq2.f = nodeq2.g + h(nodeq2);  // f=g+h

    neighbors.push_back(nodeq2);
    return neighbors;
  }*/

/*      std::cout << "Size cvxhull=" << hulls_[obst_index][current.index + 1 - 3].size() << std::endl;

      std::cout << "Vertexes obstacle" << std::endl;
      for (auto vertex_obs : hulls_[obst_index][current.index + 1 - 3])
      {
        std::cout << "vertex_obs= " << vertex_obs.transpose() << std::endl;
      }

      std::cout << "CPs" << std::endl;
      for (auto cp : last4Cps)
      {
        std::cout << "cp= " << cp.transpose() << std::endl;
      }

      std::cout << "n_i= " << n_i.transpose() << ", d_i= " << d_i << std::endl;*/
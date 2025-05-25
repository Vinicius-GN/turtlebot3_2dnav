/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file rrt_planning/src/vertex.cc
 *
 * @brief Small class to maintain information held in vertices
 *
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 #include "/home/vinicius/catkin_ws/src/turtlebot3_2dnav/include/rrt_planner/vertex.h"

 namespace rrt_planning {
   Vertex::Vertex(float x, float y, int index, int parent_index) {
     x_ = x;
     y_ = y;
     index_ = index;
     parent_index_ = parent_index;
   }
 
   void Vertex::set_location(float x, float y) {
     x_ = x;
     y_ = y;
   }
 
   void Vertex::set_index(int index) {
     index_ = index;
   }
 
   void Vertex::set_parent(int parent_index) {
     parent_index_ = parent_index;
   }
 
   std::pair<float, float> Vertex::get_location() {
     return std::pair<float, float>(x_, y_);
   }
 
   int Vertex::get_index() {
     return index_;
   }
 
   int Vertex::get_parent() {
     return parent_index_;
   }
 }  // namespace rrt_planning
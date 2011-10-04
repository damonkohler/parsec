// Copyright 2011 Google Inc.
// Author: whess@google.com (Wolfgang Hess)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PARSECLIB_SAFETY_
#define PARSECLIB_SAFETY_

// Simply an orthogonal projection of velocity to the plane supported by
// unsafe_direction when necessary, i.e., the part of velocity that is
// orthogonal to the unsafe_direction remains, the other part only if it
// is not in the unsafe_direction.
template<int dim>
void TrimForSafety(
    float *velocity, const float *unsafe_direction) {
  float u_dot_u = 0.0f;
  float v_dot_u = 0.0f;
  for (int i = 0; i != dim; ++i) {
    u_dot_u += unsafe_direction[i] * unsafe_direction[i];
    v_dot_u += velocity[i] * unsafe_direction[i];
  }
  float factor = v_dot_u / u_dot_u;
  if (factor > 0.0f) {  // Velocity is in the unsafe direction.
    for (int i = 0; i != dim; ++i) {
      velocity[i] -= factor * unsafe_direction[i];
    }
  }
}

template<>
void TrimForSafety<1>(
    float *velocity, const float *unsafe_direction) {
  if ((*velocity < 0.0f) == (*unsafe_direction < 0)) {
    *velocity = 0.0f;
  }
}

#endif  // PARSECLIB_SAFETY_


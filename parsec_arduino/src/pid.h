// Copyright 2011 Google Inc.
// Author: moesenle@google.com (Lorenz Moesenlechner)
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

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class Pid {
 public:
  Pid()
    : p_(0.0f), i_(0.0f), d_(0.0f), i_clamp_(0.0f),
      p_error_(0.0f), p_error_last_(0.0f),
      d_error_(0.0f), i_error_(0.0f),
      last_cmd_(0.0f) {}

  /**
   * @param i_clamp the maximal i term used for the controller
   */
  Pid(float p, float i, float d, float i_clamp)
    : p_(p), i_(i), d_(d), i_clamp_(i_clamp),
      p_error_(0.0f), p_error_last_(0.0f),
      d_error_(0.0f), i_error_(0.0f),
      last_cmd_(0.0f) {}

  /**
   * Resets the PID controller, i.e. sets all errors to 0
   */
  void reset() {
    last_cmd_ = p_error_ = p_error_last_ = d_error_ = i_error_ = 0.0f;
  }

  /**
   * Get the current gains
   */
  void gains(float *p, float *i, float *d, float *i_clamp) {
    *p = p_;
    *i = i_;
    *d = d_;
    *i_clamp = i_clamp_;
  }

  /**
   * Set the gains.
   */
  void setGains(float p, float i, float d, float i_clamp) {
    p_ = p;
    i_ = i;
    d_ = d;
    i_clamp_ = i_clamp;
  }

  /**
   * Updates the controller based on the current value, the goal and a
   * time step. Returns a new control command.
   */
  float update(float current_value, float goal, float dt) {
    p_error_last_ = p_error_;
    p_error_ = goal - current_value;

    i_error_ += dt * p_error_;
    if (i_error_ > i_clamp_) {
      i_error_ = i_clamp_;
    } else if (i_error_ < -i_clamp_) {
      i_error_ = -i_clamp_;
    }

    d_error_ = (p_error_ - p_error_last_) / dt;

    last_cmd_ =  p_ * p_error_ + i_ * i_error_ + d_ * d_error_;
    return last_cmd_;
  }

  /**
   * Returns the last command that has been calculated by update.
   */
  float lastCmd() { return last_cmd_; }

  /**
   * Returns the last error between the current position and goal as
   * passed to the update method.
   */
  float error() { return p_error_; }

 private:
  float p_;
  float i_;
  float d_;
  float i_clamp_;

  float last_cmd_;
  float p_error_;
  float p_error_last_;
  float d_error_;
  float i_error_;
};

#endif  // PID_CONTROLLER_H

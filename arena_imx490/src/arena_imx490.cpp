/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 *****************************************************************************/

// STD
#include <string>
#include <vector>

// Arena
#include <ArenaApi.h>

// Arena node
#include <arena_imx490/internal/arena_imx490.h>

namespace arena_imx490 {

ArenaIMX490::ArenaIMX490()
    : device_user_id_(""),
      img_rows_(0),
      img_cols_(0),
      img_size_byte_(0),
      grab_timeout_(-1.0),
      is_ready_(false),
      max_brightness_tolerance_(2.5) {}

const std::string& ArenaIMX490::deviceUserID() const { return device_user_id_; }

const size_t& ArenaIMX490::imageRows() const { return img_rows_; }

const size_t& ArenaIMX490::imageCols() const { return img_cols_; }

const size_t& ArenaIMX490::imageSize() const { return img_size_byte_; }

const float& ArenaIMX490::maxBrightnessTolerance() const {
  return max_brightness_tolerance_;
}

const bool& ArenaIMX490::isReady() const { return is_ready_; }

std::size_t ArenaIMX490::numUserOutputs() const {
  return user_output_selector_enums_.size();
}

const std::vector<float>& ArenaIMX490::sequencerExposureTimes() const {
  return seq_exp_times_;
}

ArenaIMX490::~ArenaIMX490() {
  // Releases all Arena resources.
}

}  // namespace arena_imx490

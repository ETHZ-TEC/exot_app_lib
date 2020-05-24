// Copyright (c) 2015-2020, Swiss Federal Institute of Technology (ETH Zurich)
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
/**
 * @file framework/all.h
 * @author     Bruno Klopott
 * @brief      Header file combining the core framework and defining convienient
 *             aliases.
 */

#pragma once

#include <exot/framework/connect.h>
#include <exot/framework/executor.h>
#include <exot/framework/interface.h>
#include <exot/framework/node.h>
#include <exot/framework/queue.h>
#include <exot/framework/state.h>

namespace exot::framework {

/**
 * Aliases for nodes using thread-based interfaces.
 */
template <typename InputToken>
using ThreadConsumer =
    IConsumer<InputToken, TimeoutLockingQueue, ExtendedQueueReader>;

template <typename OutputToken>
using ThreadProducer =
    IProducer<OutputToken, TimeoutLockingQueue, ExtendedQueueWriter>;

template <typename InputToken, typename OutputToken>
using ThreadProcessor =
    IProcessor<InputToken, TimeoutLockingQueue, ExtendedQueueReader,
               OutputToken, TimeoutLockingQueue, ExtendedQueueWriter>;

/**
 * Aliases for basic node types.
 */
template <typename InputToken>
using Consumer = ThreadConsumer<InputToken>;

template <typename OutputToken>
using Producer = ThreadProducer<OutputToken>;

template <typename InputToken, typename OutputToken>
using Processor = ThreadProcessor<InputToken, OutputToken>;

#ifdef EXOT_USE_FIBERS

/**
 * Aliases for nodes using fiber-based interfaces.
 */
template <typename InputToken>
using FiberConsumer =
    IConsumer<InputToken, FiberTimeoutLockingQueue, ExtendedQueueReader>;

template <typename OutputToken>
using FiberProducer =
    IProducer<OutputToken, FiberTimeoutLockingQueue, ExtendedQueueWriter>;

template <typename InputToken, typename OutputToken>
using FiberProcessor =
    IProcessor<InputToken, FiberTimeoutLockingQueue, ExtendedQueueReader,
               OutputToken, FiberTimeoutLockingQueue, ExtendedQueueWriter>;

#endif

}  // namespace exot::framework

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

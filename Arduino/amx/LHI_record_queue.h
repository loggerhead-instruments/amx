/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef record_queue_h_
#define record_queue_h_

#include "AudioStream.h"
#ifndef MQ
#define MQ 53
#endif

class LHIRecordQueue : public AudioStream
{
public:
  LHIRecordQueue(void) : AudioStream(1, inputQueueArray),
    userblock(NULL), head(0), tail(0), enabled(0), queue_dropped(0){ }
  void begin(void) {
    clear();
    enabled = 1;
  }
  int available(void);
  void clear(void);
  int16_t * readBuffer(void);
  void freeBuffer(void);
  void end(void) {
    enabled = 0;
  }
  virtual void update(void);
private:
  audio_block_t *inputQueueArray[1];
  audio_block_t * volatile queue[MQ];
  audio_block_t *userblock;
  volatile uint16_t head, tail, enabled;
  uint32_t queue_dropped;
public:
  uint32_t getQueue_dropped(void) {return queue_dropped;}
  void clearQueue_dropped(void) {queue_dropped=0;}
};

int LHIRecordQueue::available(void)
{
  uint32_t h, t;

  h = head;
  t = tail;
  if (h >= t) return h - t;
  return MQ + h - t;
}

void LHIRecordQueue::clear(void)
{
  uint32_t t;

  if (userblock) {
    release(userblock);
    userblock = NULL;
  }
  t = tail;
  while (t != head) {
    if (++t >= MQ) t = 0;
    release(queue[t]);
  }
  tail = t;
}

int16_t * LHIRecordQueue::readBuffer(void)
{
  uint32_t t;

  if (userblock) return NULL;
  t = tail;
  if (t == head) return NULL;
  if (++t >= MQ) t = 0;
  userblock = queue[t];
  tail = t;
  return userblock->data;
}

void LHIRecordQueue::freeBuffer(void)
{
  if (userblock == NULL) return;
  release(userblock);
  userblock = NULL;
}

void LHIRecordQueue::update(void)
{
  audio_block_t *block;
  uint32_t h;
  
  block = receiveReadOnly();
  if (!block) return;
  if (!enabled) {
    release(block);
    return;
  }
  h = head + 1;
  if (h >= MQ) h = 0;
  if (h == tail) {
    queue_dropped++;
    release(block);
  } else {
    queue[h] = block;
    head = h;
  }
}

#endif



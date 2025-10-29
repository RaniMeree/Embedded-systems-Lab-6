# Task1a Problem Demonstration and Solution

## 🔴 PROBLEM: Race Condition on `byteCount`

### The Issue

**`byteCount` is a SHARED VARIABLE accessed by BOTH tasks without protection!**

Look at these critical lines:

**Producer (line 113):**
```c
byteCount = byteCount + 1;  // NOT ATOMIC!
```

**Consumer (line 141):**
```c
byteCount = byteCount - 1;  // NOT ATOMIC!
```

### Why This is Dangerous

The operation `byteCount = byteCount + 1` is **NOT atomic** - it's actually 3 steps:
1. **READ** current value of `byteCount`
2. **ADD** 1 to it
3. **WRITE** result back to `byteCount`

If Producer and Consumer run at the same time, they can interfere!

---

## 💥 Race Condition Scenario

### Initial State
- `byteCount = 5`
- Producer and Consumer both running

### Bad Timing Sequence

| Time | Producer | Consumer | byteCount Value |
|------|----------|----------|-----------------|
| **1** | Read byteCount (= 5) | | 5 |
| **2** | | Read byteCount (= 5) | 5 |
| **3** | Add 1 → result = 6 | | 5 |
| **4** | | Subtract 1 → result = 4 | 5 |
| **5** | Write 6 to byteCount | | **6** ❌ |
| **6** | | Write 4 to byteCount | **4** ❌ |

### Result: WRONG!
- **Expected:** byteCount should be 5 (one added, one removed)
- **Actual:** byteCount = 4 (lost the producer's increment!)

This is called a **"lost update"** problem.

---

## 🐛 Additional Problems

### Problem 2: Check-Then-Act Race Condition

**Producer (lines 105-110):**
```c
if (byteCount == BUFFER_SIZE)  // CHECK
{
    sleep();                    // ACT (but gap between check and act!)
}
```

**What can go wrong:**
1. Producer checks: `byteCount == 10` (TRUE, buffer full)
2. **INTERRUPT!** Consumer runs and removes item → `byteCount = 9`
3. Producer goes to sleep (but buffer is NOT full anymore!)
4. **DEADLOCK!** Producer sleeping, consumer might also sleep, nobody wakes anyone!

---

## 🎯 DEMONSTRATION: How to See the Bug

### Test Scenario

1. **Remove the delays** (you already did this! Good!)
   ```c
   // vTaskDelay(pdMS_TO_TICKS(100));  // Commented out
   ```

2. **Run the program** and watch for:
   - `byteCount` becoming **negative** (impossible in correct logic!)
   - `byteCount` exceeding `BUFFER_SIZE` (also impossible!)
   - Tasks **deadlocking** (both sleeping, neither waking)
   - Incorrect buffer state

3. **Typical buggy output:**
   ```
   [PRODUCER] Added 1 to buffer[5] (now = 1)
     Buffer: [1 1 1 1 1 1  0  0  0  0] byteCount=6

   [CONSUMER] Subtracted 1 from buffer[0] (now = 0)
     Buffer: [0 1 1 1 1 1  0  0  0  0] byteCount=6  ← WRONG! Should be 5!

   [PRODUCER] Added 1 to buffer[6] (now = 1)
     Buffer: [0 1 1 1 1 1  1  0  0  0] byteCount=7

   [CONSUMER] Subtracted 1 from buffer[1] (now = 0)
     Buffer: [0 0 1 1 1 1  1  0  0  0] byteCount=7  ← WRONG! Should be 6!
   ```

---

## ✅ SOLUTION: Add Mutual Exclusion (Mutex)

### Option 1: Use a Mutex/Semaphore (Best Solution)

Add mutual exclusion to protect `byteCount`:

```c
static SemaphoreHandle_t mutex;

// In ProducerTask:
xSemaphoreTake(mutex, portMAX_DELAY);  // LOCK
byteCount = byteCount + 1;              // SAFE
xSemaphoreGive(mutex);                  // UNLOCK

// In ConsumerTask:
xSemaphoreTake(mutex, portMAX_DELAY);  // LOCK
byteCount = byteCount - 1;              // SAFE
xSemaphoreGive(mutex);                  // UNLOCK
```

### Option 2: Disable Interrupts (Not Recommended)

```c
taskENTER_CRITICAL();
byteCount = byteCount + 1;
taskEXIT_CRITICAL();
```

**Why not recommended?** Disabling interrupts affects system responsiveness.

### Option 3: Atomic Operations (If Available)

Some processors have atomic increment/decrement instructions, but they're not guaranteed in all FreeRTOS ports.

---

## 📊 Comparison: Before vs After Fix

| Aspect | Without Mutex (Current) | With Mutex (Fixed) |
|--------|------------------------|-------------------|
| **byteCount accuracy** | ❌ Can be wrong | ✅ Always correct |
| **Lost updates** | ❌ Possible | ✅ Prevented |
| **Check-then-act races** | ❌ Possible | ✅ Prevented |
| **Deadlock risk** | ❌ High | ✅ Low |
| **Code complexity** | ✅ Simple | ⚠️ Slightly more complex |

---

## 🔧 Implementation: Fixed Version

See `task1b.c` for the complete solution using semaphores!

Key differences in task1b:
1. **Mutex** protects buffer access
2. **Counting semaphores** (`emptySlots`, `fullSlots`) handle synchronization
3. **No race conditions** on `byteCount` (not even needed - semaphores handle it!)

---

## 🎓 Key Takeaways

1. **Shared variables need protection** - Always use mutex/semaphore
2. **Non-atomic operations are dangerous** - `x = x + 1` is 3 operations!
3. **Check-then-act is risky** - Gap between check and action can cause bugs
4. **sleep()/wakeup() alone is NOT enough** - You still need mutual exclusion!
5. **Task1b shows the correct way** - Use semaphores for synchronization

---

## 🧪 Suggested Testing

1. **Run task1a** - Watch it fail with race conditions
2. **Add debug output** - Print byteCount frequently to see corruption
3. **Run task1b** - See how semaphores fix everything
4. **Compare outputs** - Task1a chaotic, task1b stable

The problem demonstrates why **synchronization primitives** (mutexes, semaphores) are essential in concurrent programming!



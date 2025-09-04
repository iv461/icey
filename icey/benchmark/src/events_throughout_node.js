const TOTAL_EVENTS = 1_000_000; // one million
let completed = 0;

// Small event function
function eventTask() {
    completed++;
    if (completed === TOTAL_EVENTS) {
        const end = process.hrtime.bigint();
        const durationNs = end - start;
        const durationMs = Number(durationNs) / 1e6;
        console.log(`Processed ${TOTAL_EVENTS.toLocaleString()} events in ${durationMs.toFixed(2)} ms`);
        console.log(`Throughput: ${(TOTAL_EVENTS / (durationMs / 1000)).toFixed(0)} events/sec`);
    }
}

let start;


console.log(`Launching ${TOTAL_EVENTS.toLocaleString()} events...`);

start = process.hrtime.bigint();

// Queue up 1,000,000 tasks in the event loop
for (let i = 0; i < TOTAL_EVENTS; i++) {
    setImmediate(eventTask); // schedules quickly without blowing the stack
}

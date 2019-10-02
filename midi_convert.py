#!/usr/bin/python3
from mido import MidiFile
import math

has_note = [False] * 4
note_sets = [list() for i in range(4)]
note_time_sets = [list() for i in range(4)]
delay_time_sets = [list() for i in range(4)]
last_times = [0.0] * 4

current_time = 0.0

def open_set():
    for i in range(4):
        if not has_note[i]:
            return i
    print("BOOM!")
    import pdb; pdb.set_trace()
    exit(1)

def set_of_note(note):
    for i in range(4):
        if note_sets[i][-1] == note:
            return i
    print("BOOM!")
    import pdb; pdb.set_trace()
    exit(1)

ticks = 0

delay_factor = 20

mid = MidiFile("Mario-Sheet-Music-Overworld-Main-Theme.mid")
for i, track in enumerate(mid.tracks):
    current_time = 0.0
    last_times = [0.0] * 4

    for msg in track:
        if msg.is_meta:
            continue

        current_time += msg.time

        if msg.type == "note_on":
            set_i = open_set()

            # delay times can be much bigger sometimes than 2550 (will be 255)
            # add in blank note entries to keep the delays small enough
            delay_time = current_time - last_times[set_i]
            while delay_time > 255*delay_factor:
                delay_time_sets[set_i] += [255*delay_factor]
                note_sets[set_i] += [0]
                delay_time -= 255*delay_factor
                if delay_time > 255*delay_factor:
                    note_time_sets[set_i] += [255*delay_factor]
                else:
                    note_time_sets[set_i] += [0]


            delay_time_sets[set_i] += [delay_time]
            note_sets[set_i] += [msg.note]
            last_times[set_i] = current_time
            has_note[set_i] = True
        elif msg.type == "note_off":
            set_i = set_of_note(msg.note)
            note_time_sets[set_i] += [current_time - last_times[set_i]]
            last_times[set_i] = current_time
            has_note[set_i] = False

min_note = 1000
max_note = 0

for i in range(4):
    print("const uint8_t notes_{}[{}] = {{".format(i, len(note_sets[i])), end='')
    for j, note in enumerate(note_sets[i]):
        max_note = max(max_note, note)
        if note > 0:
            min_note = min(min_note, note)
        if j > 0:
            print(", ", end='')
        print("{}".format(note), end='')
    print("};")

    print("const uint8_t delays_{}[{}] = {{".format(i, len(delay_time_sets[i])), end='')
    for j, delay in enumerate(delay_time_sets[i]):
        if j > 0:
            print(", ", end='')
        print("{}".format(int(delay / delay_factor)), end='')
    print("};")

    print("const uint8_t times_{}[{}] = {{".format(i, len(note_time_sets[i])), end='')
    for j, time in enumerate(note_time_sets[i]):
        if j > 0:
            print(", ", end='')
        print("{}".format(int(time / delay_factor)), end='')
    print("};")

print("const uint8_t notes_to_freqs_offset = {};".format(min_note))
print("const uint16_t notes_to_freqs[{}] = {{".format(max_note - min_note + 1), end='')
for note in range(min_note, max_note + 1):
    if note > min_note:
        print(", ", end='')
    print("{}".format(int(2 * 440 * math.pow(2, (note - 69) / 12) + 0.5)), end='')
print("};")

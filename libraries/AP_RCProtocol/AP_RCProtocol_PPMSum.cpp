/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol_PPMSum.h"

/*
  process a PPM-sum pulse of the given width
 */
void AP_RCProtocol_PPMSum::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    // Stardard PPM details:
    //
    //   width_s0 is the gap between the end of the last impulse
    //   and the start of the current impulse, around 1000-2000 us
    //
    //   width_s1 is the actual pulse width that is between channels.
    //
    //   The useful pulse length is the time between two falling edges,
    //   i.e., the sum of the gap and the current impulse width, s0 + s1
    //
    // Notes for Collmot's hacked FrSky v8R7II -> PPMSum receivers:
    //
    //   in our receivers width_s1 is around 90-100 us before all channels
    //   and 32 us after the 8th channel. This shows the end of the frame.
    //
    //   frames come at 18ms, so main frequency is 55.5 Hz
    //
    //   max data length within a frame is thus approximately
    //   8 * 2000us + 8 * 100us + 32us = 16832 us,
    //   so only 1168 us is left until next frame.
    //   So detection of 32us pulse is critical!!!

    // we need to get the short s1 that signals the end of the frame, so
    // we need a static variable and we also need to assume that
    // process_pulse gets called only once for every pulse.
    static uint32_t last_width_s1 = 100;

    if (width_s0 == 0 || width_s1 == 0) {
        //invalid data: reset frame
        ppm_state._channel_counter = -1;
        return;
    }
    uint32_t width_usec = width_s0 + width_s1;
    if (width_usec >= 2700 || last_width_s1 < 50) {
        // a long pulse or a short last impulse indicates the end of a frame.
        // Reset the channel counter so next pulse is channel 0
        if (ppm_state._channel_counter >= MIN_RCIN_CHANNELS) {
            add_input(ppm_state._channel_counter, ppm_state._pulse_capt, false);
        }
        ppm_state._channel_counter = 0;
        last_width_s1 = width_s1;
        return;
    }

    last_width_s1 = width_s1;

    if (ppm_state._channel_counter == -1) {
        // we are not synchronised
        return;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

        // move to next channel
        ppm_state._channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (ppm_state._channel_counter >= MAX_RCIN_CHANNELS) {
        add_input(ppm_state._channel_counter, ppm_state._pulse_capt, false);
        ppm_state._channel_counter = -1;
    }
}

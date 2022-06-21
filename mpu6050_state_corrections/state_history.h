#ifndef STATE_HISTORY_H
#define STATE_HISTORY_H

class StateHistory {
    public:
        int buff_size;
        int i;
        int* state_buff;
        bool was_buff_filled;
        int current_state;
        int current_correction;
        StateHistory(int buff_size) : buff_size(buff_size), i(0), was_buff_filled(false), current_state(-1), current_correction(-1) {
            state_buff = new int[buff_size]; }

        ~StateHistory() {
            delete[] state_buff;
        }

        void update_state(int new_state) {
            state_buff[i] = new_state;
            if (current_state == -1) {
                current_state = new_state;
            }
            if (current_correction == -1 ) {
                current_correction = new_state;
            }
            if (!was_buff_filled && i == buff_size - 1) {
                was_buff_filled = true;
            }
            i = (i + 1) % buff_size;
        }

        bool compare_lower_half(int state, bool any=false) {
            if (!was_buff_filled) return false;

            return compare_internal(i, state, any);
        }

        bool compare_higher_half(int state, bool any=false) {
            if (!was_buff_filled) return false;
            
            int base_i = (i - (buff_size / 2)) % buff_size;
            if (base_i < 0) base_i = -base_i;
            return compare_internal(base_i, state, any);
        }

        bool compare_internal(int base_i, int state, bool any=false) {
            int i_counter = 0;
            int j = base_i;
            int target_state = (any ? state_buff[j] : state);
            while (i_counter < buff_size / 2) {
                if (state_buff[j] != target_state) {
                    return false;
                }
                j = (j + 1) % buff_size;
                i_counter++;
            }
            return true;
        }

        //int get_prev() {
            //int prev_i = (i - 1) % buff_size;
            //if (prev_i < 0) {
                //prev_i = -prev_i;
            //}
            //return state_buff[prev_i];
        //}

        int get_oldest() {
            //int prev_i = (i) % buff_size;
            //Serial.print("current i = "); Serial.print(i); Serial.print(" oldest = "); Serial.println(prev_i);
            //if (prev_i < 0) {
                //prev_i = -prev_i;
            //}
            return state_buff[i];
        }
        
        bool has_state_changed(int next, int* prev) {
            *prev = current_state;
            if (next == *prev) {
                return false;
            }
            return compare_higher_half(next);
        }
        
        void change_state(int next_state) {
            if (current_correction == 3 && next_state != 3) {
                current_correction = 0;
            } else if (current_state != 3 && current_correction == 0 && next_state != 0) {
                current_correction = 1;
            //} else if (current_state != 3 && current_correction == 1 && next_state != 1) {
            } else if (current_correction == 1 && next_state != 1) {
                current_correction = 2;
            } else if (current_correction == 2 && next_state == 3) {
                current_correction = 3;
            }

            current_state = next_state;
        }
};

#endif

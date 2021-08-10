
#include "grbl.h"

#define MAX_LINE_NUMBER 9999999

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *Undefined but required


#define FAIL(status) return(status);


void gc_init()
{
  memset(&gc_state, 0, sizeof(gc_state));

  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) {
    report_status_message(STATUS_SETTING_READ_FAIL);
  }
}


// Sets g-code position in mm. Input in steps
void gc_sync_position()
{
  system_convert_array_steps_to_mpos(gc_state.position,sys.position);
}


static uint8_t gc_check_same_position(float *pos_a, float *pos_b)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    if (pos_a[idx] != pos_b[idx]) { return(false); }
  }
  return(true);
}

// coordinates, respectively.
uint8_t gc_execute_line(char *line)
{


  while (line[char_counter] != 0) { // Loop until no more g-code words in line.

    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value]

    int_value = trunc(value);
    mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.

    switch(letter) {

      /* 'G' and 'M' Command Words*/

      case 'G':
        // Determine 'G' command and its  group
        switch(int_value) {
          case 10: case 28: case 30: case 92:

            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            // No break. Continues to next line.
          case 4: case 53:
            word_bit = MODAL_GROUP_G0;
            switch(int_value) {
              case 4: gc_block.non_modal_command = NON_MODAL_DWELL; break; // G4
              case 10: gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_DATA; break; // G10
              case 28:
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_GO_HOME_0; break;  // G28
                  case 10: gc_block.non_modal_command = NON_MODAL_SET_HOME_0; break; // G28.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G28.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
              case 30:
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_GO_HOME_1; break;  // G30
                  case 10: gc_block.non_modal_command = NON_MODAL_SET_HOME_1; break; // G30.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G30.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
              case 53: gc_block.non_modal_command = NON_MODAL_ABSOLUTE_OVERRIDE; break; // G53
              case 92:
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_OFFSET; break; // G92
                  case 10: gc_block.non_modal_command = NON_MODAL_RESET_COORDINATE_OFFSET; break; // G92.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G92.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
            }
            break;
          case 0: case 1: case 2: case 3: case 38:
            // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE;
            // No break. Continues to next line.
          case 80:
            word_bit = MODAL_GROUP_G1;
            switch(int_value) {
              case 0: gc_block.modal.motion = MOTION_MODE_SEEK; break; // G0
              case 1: gc_block.modal.motion = MOTION_MODE_LINEAR; break; // G1
              case 2: gc_block.modal.motion = MOTION_MODE_CW_ARC; break; // G2
              case 3: gc_block.modal.motion = MOTION_MODE_CCW_ARC; break; // G3
              case 38:
                switch(mantissa) {
                  case 20: gc_block.modal.motion = MOTION_MODE_PROBE_TOWARD; break; // G38.2
                  case 30: gc_block.modal.motion = MOTION_MODE_PROBE_TOWARD_NO_ERROR; break; // G38.3
                  case 40: gc_block.modal.motion = MOTION_MODE_PROBE_AWAY; break; // G38.4
                  case 50: gc_block.modal.motion = MOTION_MODE_PROBE_AWAY_NO_ERROR; break; // G38.5
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G38.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
              case 80: gc_block.modal.motion = MOTION_MODE_NONE; break; // G80
            }
            break;
          case 17: case 18: case 19:
            word_bit = MODAL_GROUP_G2;
            switch(int_value) {
              case 17: gc_block.modal.plane_select = PLANE_SELECT_XY; break;
              case 18: gc_block.modal.plane_select = PLANE_SELECT_ZX; break;
              case 19: gc_block.modal.plane_select = PLANE_SELECT_YZ; break;
            }
            break;
          case 90: case 91:
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3;
              if (int_value == 90) { gc_block.modal.distance = DISTANCE_MODE_ABSOLUTE; } // G90
              else { gc_block.modal.distance = DISTANCE_MODE_INCREMENTAL; } // G91
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
              // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
            }
            break;
          case 93: case 94:
            word_bit = MODAL_GROUP_G5;
            if (int_value == 93) { gc_block.modal.feed_rate = FEED_RATE_MODE_INVERSE_TIME; } // G93
            else { gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN; } // G94
            break;
          case 20: case 21:
            word_bit = MODAL_GROUP_G6;
            if (int_value == 20) { gc_block.modal.units = UNITS_MODE_INCHES; }  // G20
            else { gc_block.modal.units = UNITS_MODE_MM; } // G21
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            // NOTE: Not required since cutter radius compensation is always disabled. Only here
            // to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 43: case 49:
            word_bit = MODAL_GROUP_G8;
            // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
            // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
            // all are explicit axis commands, regardless if they require axis words or not.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict] }
            axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
            if (int_value == 49) { // G49
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
            } else if (mantissa == 10) { // G43.1
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported G43.x command]
            mantissa = 0; // Set to zero to indicate valid non-integer G command.
            break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            // NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
            word_bit = MODAL_GROUP_G12;
            gc_block.modal.coord_select = int_value-54; // Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G61.1 not supported]
            // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G command]
        }
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [Unsupported or invalid Gxx.x command]
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      case 'M':

        // Determine 'M' command and its modal group
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
              case 1: break; // Optional stop not supported. Ignore.
              case 2: case 30: gc_block.modal.program_flow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset
            }
            break;
          #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
            case 4:
          #endif
          case 3: case 5:
            word_bit = MODAL_GROUP_M7;
            switch(int_value) {
              case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;
              #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
                case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;
              #endif
              case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;
            }
            break;
         #ifdef ENABLE_M7
          case 7:
         #endif
          case 8: case 9:
            word_bit = MODAL_GROUP_M8;
            switch(int_value) {
             #ifdef ENABLE_M7
              case 7: gc_block.modal.coolant = COOLANT_MIST_ENABLE; break;
             #endif
              case 8: gc_block.modal.coolant = COOLANT_FLOOD_ENABLE; break;
              case 9: gc_block.modal.coolant = COOLANT_DISABLE; break;
            }
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
        }

        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      // NOTE: All remaining letters assign values.
      default:

        /* Non-Command Words:  */
        switch(letter){
          // case 'A': // Not supported
          // case 'B': // Not supported
          // case 'C': // Not supported
          // case 'D': // Not supported
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          // case 'H': // Not supported
          case 'I': word_bit = WORD_I; gc_block.values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
          // case 'Q': // Not supported
          case 'R': word_bit = WORD_R; gc_block.values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block.values.s = value; break;
          case 'T': word_bit = WORD_T; break; // gc.values.t = int_value;
          case 'X': word_bit = WORD_X; gc_block.values.xyz[X_AXIS] = value; axis_words |= (1<<X_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block.values.xyz[Y_AXIS] = value; axis_words |= (1<<Y_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block.values.xyz[Z_AXIS] = value; axis_words |= (1<<Z_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        } 


    }
  }
  // Parsing complete!

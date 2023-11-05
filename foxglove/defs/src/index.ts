import { ros1 } from "@foxglove/rosmsg-msgs-common";

import { writeAllDatatypeMaps } from "./datatypeMaps";
import { writeMessageDefinitions } from "./messageDefinitions";

// Following paths are relative to the foxglove/msgdefs/src directory
// For the script to work correclty, the script must be run from the foxglove/msgdefs/src directory
const MSGDEFS_PATH = "../../../core/catkin_ws/src/custom_msgs/msg";
const MSGDEFS_SAVE_DIR = "../custom_msg_defs/dist";

const DATATYPE_MAPS_SAVE_DIR = "../datatype_maps/dist";
const MSG_DEFS_RELATIVE_PATH = "../../custom_msg_defs/dist";

async function main() {
  const definitionsByGroup = await writeMessageDefinitions(MSGDEFS_PATH, MSGDEFS_SAVE_DIR);

  definitionsByGroup.set("ros1", ros1);

  await writeAllDatatypeMaps(definitionsByGroup, DATATYPE_MAPS_SAVE_DIR, MSG_DEFS_RELATIVE_PATH);
}

void main();

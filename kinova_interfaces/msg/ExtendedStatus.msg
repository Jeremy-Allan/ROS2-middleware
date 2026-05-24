# Namespace: kinova_interfaces/msg/ExtendedStatus

string node_name            # Name of the reporting node (e.g., "json_parser_node")
uint8 state                 # Current operational state of the node
string status_message       # Contextual, human-readable info or logs
bool last_command_valid     # Status indicator for the most recent transaction

# State Constants
uint8 STATE_IDLE = 0
uint8 STATE_BUSY = 1
uint8 STATE_FAULT = 2

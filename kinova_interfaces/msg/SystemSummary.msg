# Namespace: kinova_interfaces/msg/SystemSummary

uint8 summary_state                        # Overall system health evaluation
kinova_interfaces/ExtendedStatus[] individual_states  # Dynamic array of all known node states

# Aggregated State Constants
uint8 SYSTEM_READY = 0
uint8 SYSTEM_BUSY = 1
uint8 SYSTEM_FAULT = 2

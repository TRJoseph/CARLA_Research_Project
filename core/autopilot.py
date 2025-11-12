from actors.vehicle import Vehicle

def toggle_autopilot(vehicles: list[Vehicle], ap_status = True) -> None:
    for v in vehicles:
        v.enable_autopilot(ap_status)

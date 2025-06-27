class BMSAgent:
    """Simple Battery Management System agent for tests and benchmarks."""

    def __init__(self, temp_limit: float) -> None:
        self.temp_limit = temp_limit
        self.temps: list[float] = []
        self.contactor_open = False

    def ingest(self, temp: float) -> None:
        self.temps.append(temp)
        if self.check_trip():
            self.open_contactor()

    def check_trip(self) -> bool:
        if not self.temps:
            return False
        return self.temps[-1] > self.temp_limit

    def open_contactor(self) -> None:
        self.contactor_open = True
        print("CONTACTOR OPEN")

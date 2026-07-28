"""Transport abstraction isolating guidance from UDP, serial, and ELRS."""

from typing import Protocol


class MavlinkTransport(Protocol):
    async def connect(self) -> None:
        """Open the configured transport."""
        ...

    async def receive(self) -> object:
        """Receive one MAVLink message."""
        ...

    async def send(self, message: object) -> None:
        """Send one MAVLink message."""
        ...

    async def close(self) -> None:
        """Close the transport and release resources."""
        ...


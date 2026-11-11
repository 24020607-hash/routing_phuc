
####################################################
# DVrouter.py
# Name:HOANG TUAN PHUC
# HUID:24020607
#####################################################

import json
import math

from router import Router
from packet import Packet


class DVrouter(Router):
    """Distance Vector routing using Bellman-Ford."""

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)

        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # port -> (neighbor address, link cost)
        self.neighbors = {}

        # port -> advertised distance vector
        self.neighbor_vectors = {}

        # destination -> minimum cost
        self.distance_vector = {self.addr: 0}

        # destination -> outgoing port
        self.forwarding_table = {}

    def broadcast(self):
        """Send distance vector to all neighbors."""
        for port, (neighbor, cost) in self.neighbors.items():

            # Poison reverse helps prevent routing loops.
            vector = dict(self.distance_vector)

            for dest, next_port in self.forwarding_table.items():
                if next_port == port and dest != neighbor:
                    vector[dest] = math.inf

            packet = Packet(
                Packet.ROUTING,
                self.addr,
                neighbor,
                json.dumps(vector)
            )

            self.send(port, packet)

    def update_routes(self):
        """Recompute shortest paths using Bellman-Ford."""

        new_vector = {self.addr: 0}
        new_table = {}

        # Directly connected destinations
        for port, (neighbor, cost) in self.neighbors.items():
            if neighbor == self.addr:
                continue

            if cost < new_vector.get(neighbor, math.inf):
                new_vector[neighbor] = cost
                new_table[neighbor] = port

        # Routes learned from neighbors
        for port, (neighbor, link_cost) in self.neighbors.items():

            vector = self.neighbor_vectors.get(port, {})

            for dest, advertised_cost in vector.items():

                if dest == self.addr:
                    continue

                if not isinstance(advertised_cost, (int, float)):
                    continue

                total_cost = link_cost + advertised_cost

                if total_cost < new_vector.get(dest, math.inf):
                    new_vector[dest] = total_cost
                    new_table[dest] = port

        changed = (
            new_vector != self.distance_vector
            or new_table != self.forwarding_table
        )

        self.distance_vector = new_vector
        self.forwarding_table = new_table

        if changed:
            self.broadcast()

    def handle_packet(self, port, packet):
        """Process traceroute and routing packets."""

        if packet.is_traceroute:

            dest = packet.dst_addr

            if dest in self.forwarding_table:
                out_port = self.forwarding_table[dest]
                self.send(out_port, packet)

        elif packet.is_routing:

            if port not in self.neighbors:
                return

            try:
                vector = json.loads(packet.content)
            except (ValueError, TypeError):
                return

            if not isinstance(vector, dict):
                return

            if vector != self.neighbor_vectors.get(port):
                self.neighbor_vectors[port] = vector
                self.update_routes()

    def handle_new_link(self, port, endpoint, cost):
        """Handle a new link."""

        self.neighbors[port] = (endpoint, cost)
        self.neighbor_vectors.pop(port, None)

        self.update_routes()
        self.broadcast()

    def handle_remove_link(self, port):
        """Handle a removed link."""

        self.neighbors.pop(port, None)
        self.neighbor_vectors.pop(port, None)

        self.update_routes()
        self.broadcast()

    def handle_time(self, time_ms):
        """Periodically broadcast distance vector."""

        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast()

    def __repr__(self):
        return (
            f"DVrouter(addr={self.addr}, "
            f"distance_vector={self.distance_vector}, "
            f"forwarding_table={self.forwarding_table})"
        )


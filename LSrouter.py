
####################################################
# LSrouter.py
# Name:HOANG TUAN PHUC
# HUID:24020607
#####################################################

import json
import heapq
import math

from router import Router
from packet import Packet


class LSrouter(Router):
    """Link State routing protocol using Dijkstra."""

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)

        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # port -> (neighbor address, link cost)
        self.neighbors = {}

        # Link State Database
        # router -> {"seq": number, "links": {neighbor: cost}}
        self.lsdb = {}

        # Sequence number for local link state
        self.seq = 0

        # destination -> outgoing port
        self.forwarding_table = {}

        self.update_local_state()

    def update_local_state(self):
        """Update this router's link state."""

        links = {}

        for port, (neighbor, cost) in self.neighbors.items():
            if neighbor not in links or cost < links[neighbor]:
                links[neighbor] = cost

        self.seq += 1

        self.lsdb[self.addr] = {
            "seq": self.seq,
            "links": links
        }

    def make_packet(self, origin):
        """Create a link state advertisement."""

        state = self.lsdb[origin]

        content = json.dumps({
            "origin": origin,
            "seq": state["seq"],
            "links": state["links"]
        })

        return Packet(
            Packet.ROUTING,
            self.addr,
            origin,
            content
        )

    def flood(self, origin, exclude_port=None):
        """Flood an LSA to neighboring routers."""

        for port in self.neighbors:

            if port == exclude_port:
                continue

            packet = self.make_packet(origin)
            self.send(port, packet)

    def run_dijkstra(self):
        """Compute shortest paths from this router."""

        graph = {}

        for router, state in self.lsdb.items():
            graph[router] = dict(state["links"])

        # Ensure direct links are represented.
        graph[self.addr] = dict(
            self.lsdb[self.addr]["links"]
        )

        distances = {self.addr: 0}
        first_hop = {}

        # (distance, destination)
        pq = [(0, self.addr)]

        while pq:
            current_dist, current = heapq.heappop(pq)

            if current_dist > distances.get(current, math.inf):
                continue

            for neighbor, cost in graph.get(current, {}).items():

                new_dist = current_dist + cost

                if new_dist < distances.get(neighbor, math.inf):

                    distances[neighbor] = new_dist

                    if current == self.addr:
                        first_hop[neighbor] = neighbor
                    else:
                        first_hop[neighbor] = first_hop[current]

                    heapq.heappush(
                        pq, (new_dist, neighbor)
                    )

        # Convert first-hop addresses into ports.
        new_table = {}

        for dest, hop in first_hop.items():

            best_port = None
            best_cost = math.inf

            for port, (neighbor, cost) in self.neighbors.items():

                if neighbor == hop and cost < best_cost:
                    best_port = port
                    best_cost = cost

            if best_port is not None:
                new_table[dest] = best_port

        self.forwarding_table = new_table

    def handle_packet(self, port, packet):
        """Process incoming packets."""

        if packet.is_traceroute:

            dest = packet.dst_addr

            if dest in self.forwarding_table:
                self.send(
                    self.forwarding_table[dest],
                    packet
                )

        elif packet.is_routing:

            try:
                data = json.loads(packet.content)

                origin = data["origin"]
                seq = data["seq"]
                links = data["links"]

                if not isinstance(origin, str):
                    return

                if not isinstance(seq, int):
                    return

                if not isinstance(links, dict):
                    return

                if origin == self.addr:
                    return

                old_state = self.lsdb.get(origin)

                # Accept only newer advertisements.
                if old_state is not None:
                    if seq <= old_state["seq"]:
                        return

                self.lsdb[origin] = {
                    "seq": seq,
                    "links": links
                }

                self.run_dijkstra()

                # Forward LSA to other neighbors.
                self.flood(origin, exclude_port=port)

            except (ValueError, TypeError, KeyError):
                return

    def handle_new_link(self, port, endpoint, cost):
        """Handle a new link."""

        self.neighbors[port] = (endpoint, cost)

        self.update_local_state()
        self.run_dijkstra()

        # Advertise the updated local topology.
        self.flood(self.addr)

        # Synchronize known LSAs with new neighbor.
        for origin in list(self.lsdb):
            if origin != self.addr:
                packet = self.make_packet(origin)
                self.send(port, packet)

    def handle_remove_link(self, port):
        """Handle a removed link."""

        self.neighbors.pop(port, None)

        self.update_local_state()
        self.run_dijkstra()

        self.flood(self.addr)

    def handle_time(self, time_ms):
        """Periodically advertise local link state."""

        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms

            self.update_local_state()
            self.run_dijkstra()
            self.flood(self.addr)

    def __repr__(self):
        return (
            f"LSrouter(addr={self.addr}, "
            f"lsdb={self.lsdb}, "
            f"forwarding_table={self.forwarding_table})"
        )

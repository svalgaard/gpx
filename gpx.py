#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import sys
import re
import datetime
import lxml.etree
import glob
import typing
import math

TTrack = typing.TypeVar("TTrack", bound="Track")
VERBOSE = False
TIMESTAMPS = []


def fail(*msg):
    print(f"⚠️ Error:", *msg, file=sys.stderr)
    sys.exit(1)


def log(*msg):
    if VERBOSE:
        print(*msg)


def str2dt(s: str) -> datetime:
    if m := re.match("^(\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2})(.000)?Z$", s):
        # 2025-11-01T13:40:00Z
        return datetime.datetime.strptime(m.group(1) + "Z", "%Y-%m-%dT%H:%M:%SZ")
    raise ValueError(f"Nay not a datetime: {s}")


def distance(wp1: tuple, wp2: tuple) -> float:
    """Calculate 3D distance between two waypoints (height is ignored)."""
    # Approximate radius of earth in m
    R = 6373e3

    lon1 = math.radians(wp1[0])
    lat1 = math.radians(wp1[1])
    lon2 = math.radians(wp2[0])
    lat2 = math.radians(wp2[1])

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c

    # print("Result: ", distance)
    return distance


class Track:
    def __init__(self, waypoints: list = None):
        self.tp = "GPX"
        self.waypoints = waypoints if waypoints else []  # list of (x,y,z,t)

    def append(self, xyzt: tuple):
        self.waypoints.append(xyzt)

    def __len__(self):
        return len(self.waypoints)

    def __str__(self):
        if self.hasTime():
            if self.waypoints[0][-1].date() == self.waypoints[-1][-1].date():
                tt = f"date={self.waypoints[0][-1].date()} start={self.waypoints[0][-1].time()} end={self.waypoints[-1][-1].time()}"
            else:
                tt = f"start={self.waypoints[0][-1]} end={self.waypoints[-1][-1]}"
        else:
            tt = "hasTime=False"
        return f"<Track: {tt} {len(self.waypoints)} waypoints>"

    def hasTime(self):
        if not self.waypoints:
            return False
        if self.waypoints[0][-1] and self.waypoints[-1][-1]:
            return True
        return False

    def cutafter(self, t: datetime.datetime) -> int:
        """Cut waypoints after time t. Return number of cut waypoints."""
        cuts = 0
        while self.waypoints and self.waypoints[-1][-1] and self.waypoints[-1][-1] > t:
            self.waypoints.pop()
            cuts += 1
        return cuts

    def timespread(self, start: datetime.datetime, end: datetime.datetime):
        span = end - start
        ss = span.total_seconds()
        dt = ss / len(self.waypoints)
        assert ss > 0
        for i, s in enumerate(self.waypoints):
            x, y, z, t = s
            t = start + datetime.timedelta(seconds=i * dt)
            self.waypoints[i] = (x, y, z, t)

    @staticmethod
    def from_direct_route_between_points(js: dict, timestamps: list) -> TTrack:
        """Direct route from o-track.dk params_file.json file"""
        output = Track()

        for i, pos in enumerate(js["mapItems"]):
            name = pos.get("name", "")
            pp = pos.get("positionedItems")[0].get("positions")[0]
            t = timestamps.pop(0) if timestamps else None
            xyzt = pp.get("x"), pp.get("y"), pp.get("z"), t
            output.append(xyzt)
            log(f"Waypoint #{i}: {name}: {xyzt}")

        return output

    @staticmethod
    def from_user_positions(js: dict) -> TTrack:
        """Route from o-track.dk single user positions.json"""
        output = Track()

        for pp in js["positions"]:
            x, y, z, t = pp
            t = datetime.datetime.fromtimestamp(t / 1000, datetime.UTC) if t else None
            xyzt = x, y, z, t
            output.append(xyzt)

        return output

    @staticmethod
    def from_gpx(gpxfn: str) -> TTrack:
        """Route from GPX file
        <trkpt lat="55.0123456789" lon="10.0123456789">
            <ele>0</ele>
            <time>2025-11-01T13:40:00Z</time>
        </trkpt>
        """
        output = Track()

        xml = lxml.etree.parse(open(gpxfn))
        pts = xml.xpath(
            "//gpx:trkpt", namespaces={"gpx": "http://www.topografix.com/GPX/1/1"}
        )
        for pt in pts:
            x = float(pt.attrib["lon"])
            y = float(pt.attrib["lat"])
            z = t = None
            for c in pt.getchildren():
                if c.tag.endswith("ele"):
                    z = float(c.text)
                elif c.tag.endswith("time"):
                    t = str2dt(c.text)  # 2025-11-01T13:41:41.000Z
                elif c.tag.endswith("extensions"):
                    pass  # ignore (Garmin extension)
                else:
                    # ⚠️ Error: Unknown trkpt subtag: {http://www.topografix.com/GPX/1/1}extensions
                    fail(f"Unknown trkpt subtag: {c.tag}")
            xyzt = x, y, z, t
            output.append(xyzt)

        if not output.waypoints:
            fail(f"{gpxfn}: No waypoints found?")
            sys.exit(1)

        return output

    def fix_distance(self, actually_fix: bool = False):
        """Fix distances by recalculating  values based on 3D distance between points."""
        from math import sqrt

        if len(self.waypoints) < 2:
            return

        result = []

        for i in range(len(self.waypoints) - 1):
            dist = distance(self.waypoints[i], self.waypoints[i + 1])
            if dist < 10:
                result.append(self.waypoints[i])
                continue
            print(f"Distance between point #{i} and #{i+1} is {dist} m")
            dx, dy, dz = (
                self.waypoints[i + 1][0] - self.waypoints[i][0],
                self.waypoints[i + 1][1] - self.waypoints[i][1],
                self.waypoints[i + 1][2] - self.waypoints[i][2],
            )
            dt = (
                self.waypoints[i + 1][-1] - self.waypoints[i][-1]
                if self.waypoints[i + 1][-1] and self.waypoints[i][-1]
                else None
            )

            steps = int(dist // 9)
            for step in range(steps):
                factor = (step + 1) / steps
                x = self.waypoints[i][0] + dx * factor
                y = self.waypoints[i][1] + dy * factor
                z = self.waypoints[i][2] + dz * factor
                if dt:
                    t = self.waypoints[i][-1] + datetime.timedelta(
                        seconds=dt.total_seconds() * factor
                    )
                else:
                    t = None
                result.append((x, y, z, t))

        result.append(self.waypoints[-1])
        dsteps = len(result) - len(self.waypoints)
        if actually_fix:
            log(f"Added {dsteps} waypoints to ensure max length 10m.")
            self.waypoints = result
        else:
            log(f"Would have added {dsteps} waypoints to ensure max length 10m.")

    def write_gpx(self, gpxfn: str):
        with open(gpxfn, "w", encoding="utf-8") as outfile:
            outfile.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            outfile.write(
                """<gpx
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xmlns="http://www.topografix.com/GPX/1/1"
    xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd"
    version="1.1"
    creator="Me">\n"""
            )
            outfile.write("<trk>\n")
            outfile.write("<trkseg>\n")
            for wp in self.waypoints:
                x, y, z, t = wp
                if t:
                    t = t.strftime("%Y-%m-%dT%H:%M:%SZ")
                    tt = f"    <time>{t}</time>\n"
                else:
                    tt = ""
                outfile.write(
                    f"""<trkpt lon="{x}" lat="{y}">
    <ele>{z}</ele>
{tt}</trkpt>\n"""
                )
            outfile.write("</trkseg>\n")
            outfile.write("</trk>\n")
            outfile.write("</gpx>\n")


def convert_timestamps(tss: list) -> list[datetime.datetime]:
    result = []
    last = None
    for i, ts in enumerate(tss):
        ts = ts.strip()
        if not ts:
            continue
        if m := re.match("^(\\d{1,3})[.:](\\d{2})$", ts):
            # 1:23 or 12.34
            if last is None:
                fail(f"Error: Timestamp line '{ts}' found before a full timestamp!")
            m, s = map(int, m.groups())
            dt = last + datetime.timedelta(minutes=m, seconds=s)
        else:
            try:
                # 2025-11-01T13:40:00Z
                dt = last = str2dt(ts)
            except ValueError:
                fail(f"Error: {i}: Unrecognized timestamp line {ts!r}!")

        print(f"Timestamp #{i:02d}: {dt}")
        result.append(dt)

    return result


def main():
    global VERBOSE
    global TIMESTAMPS

    parser = argparse.ArgumentParser()
    parser.description = """
Merge GPX files. Convert o-track.dk JSON course file to GPX format.

{progname} -t timestamps.txt

{progname} -t timestamps.txt -i params_file.json -i garmin.gpx -f -o output.gpx

""".strip()
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose output"
    )
    parser.add_argument(
        "--timestamps", "-t", type=argparse.FileType("r"), help="File with time stamps"
    )
    parser.add_argument(
        "--leaps",
        "-l",
        action="store_true",
        help="Do NOT insert small steps to ensure max distance < 10 m between waypoints",
    )
    parser.add_argument("--output", "-o", help="Output file")
    parser.add_argument(
        "--force", "-f", action="store_true", help="Force overwrite of output file"
    )
    parser.add_argument(
        "input",
        nargs=argparse.ONE_OR_MORE,
        type=argparse.FileType("r"),
        metavar="INPUT",
        help="One or more input JSON or GPX files",
    )
    args = parser.parse_args()

    VERBOSE = args.verbose

    if args.timestamps:
        log(f"{args.timestamps.name}: Loading timestamps")
        TIMESTAMPS = convert_timestamps(args.timestamps.readlines())
        log(f"{args.timestamps.name}: Timestamps loaded:", len(TIMESTAMPS))

    gpxs = []
    for fd in args.input:
        data = fd.read()
        if data.strip().startswith("<"):
            # assume GPX
            gpx = Track.from_gpx(fd.name)
        else:
            # assume JSON
            js = json.loads(data)
            if "mapItems" in js:
                gpx = Track.from_direct_route_between_points(js, TIMESTAMPS)
            elif "positions" in js:
                gpx = Track.from_user_positions(js)
            else:
                fail(f"{fd.name}: Unknown JSON format?")
        log(f"{fd.name}: {gpx}")
        gpxs.append(gpx)

    log(f"Total input tracks: {len(gpxs)}")

    if len(gpxs) == 1:
        # combine all tracks into one
        gpx = gpxs[0]
    else:
        # ensure all tracks have timestamps
        if any(not gpx.hasTime() for gpx in gpxs):
            log("----")
            log("Distributing timestamps to tracks:")
            if not TIMESTAMPS:
                fail("Some input tracks lack timestamps. Provide timestamps with -t.")

            for i, gpx in enumerate(gpxs):
                if gpx.hasTime():
                    log(f"Track #{i}: {gpx} (already has timestamps)")
                    continue

                start = TIMESTAMPS[i] if i < len(TIMESTAMPS) else None
                if i < len(gpxs) - 1 and gpxs[i + 1].waypoints[0][-1]:
                    # not last track and next track has timestamps
                    end = gpxs[i + 1].waypoints[0][-1]
                else:
                    end = TIMESTAMPS[i + 1] if i + 1 < len(TIMESTAMPS) else None

                if not (start and end):
                    fail("Not enough timestamps to distribute to all tracks. Use -t")

                gpx.timespread(start, end)
                log(f"Track #{i}: {gpx} (timestamps distributed)")

        # all tracks have timestamps now

        # merge all tracks
        log("----")
        log("Merging all tracks into one:")
        gpx = Track()
        for i, tgpx in enumerate(gpxs):
            cuts = gpx.cutafter(tgpx.waypoints[0][-1])
            if cuts:
                log(f"Cutting {cuts} waypoints from to maintain time order wrt #{i}")
            log(f"Appending track #{i}: {tgpx}")
            gpx.waypoints.extend(tgpx.waypoints)

        log(f"Merged track: {gpx}")

    # fix distances
    gpx.fix_distance(not args.leaps)

    if not args.output:
        fail("Output file must be specified with --output/-o")

    if os.path.isfile(args.output) and not args.force:
        fail(f"Output file '{args.output}' exists! Use --force/-f to overwrite.")

    gpx.write_gpx(args.output)
    print(f"Converted to '{args.output}' successfully.")


if __name__ == "__main__":
    main()

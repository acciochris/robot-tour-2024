#!/usr/bin/env python3
import argparse
import httpx


def main():
    parser = argparse.ArgumentParser(description="Upload program to microcontroller")
    parser.add_argument("url")
    parser.add_argument("file", type=argparse.FileType("rb"))
    args = parser.parse_args()
    res = httpx.post(f"{args.url}/update", content=args.file.read())
    print(res.json())


if __name__ == "__main__":
    main()

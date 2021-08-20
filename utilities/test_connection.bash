#!/bin/bash
timeout=1
ping -c 1 -W $timeout 192.168.1.11  | grep -E -- '(statistics|received)'
ping -c 1 -W $timeout 192.168.1.12  | grep -E -- '(statistics|received)'
ping -c 1 -W $timeout 192.168.1.13  | grep -E -- '(statistics|received)'
ping -c 1 -W $timeout 192.168.1.101 | grep -E -- '(statistics|received)'
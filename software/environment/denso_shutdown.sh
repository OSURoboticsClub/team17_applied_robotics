#!/bin/bash

VBoxManage controlvm Windows acpipowerbutton

echo "Waiting for Windows to poweroff..."

until $(VBoxManage showvminfo --machinereadable Windows | grep -q ^VMState=.poweroff.)
do
  sleep 1
done

sudo poweroff

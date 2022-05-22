#!/usr/bin/env bash
roscd offbnode
git add .
git reset HEAD -- models/m1/EXPORT_GOOGLE_SAT_WM.tif
git commit -m "actualizacion"
git push origin master
andres14077
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

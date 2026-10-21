# YOLO model weights (`yolo11n.pt`) — obtained separately

The model weights are **not included in this repository**. The validated launch file
(`src/my_first_robot/launch/disaster_bot_full.launch.py`) requires the file at the
workspace root and stops with
`RuntimeError: disaster_bot_full.launch.py: required file not found: <workspace>/yolo11n.pt`
if it is missing. It does not download it. The launch file is part of the validated build and
was deliberately left unchanged.

## Exact file used in all validated runs

| Property | Value |
|---|---|
| File name / location | `yolo11n.pt` in the workspace root (next to `src/` and `disaster_site/`) |
| Size | 5,613,764 bytes |
| SHA-256 | `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |
| Metadata embedded in the file | `version: 8.2.100`, `date: 2024-09-25T21:10:26`, `license: AGPL-3.0 License (https://ultralytics.com/license)`, `docs: https://docs.ultralytics.com` |
| Model | Ultralytics YOLO11n, pretrained (COCO). Not retrained or modified. The project uses only class 0 (person) with confidence ≥ 0.50 |

**Official source match (checked 2026-10-14, HTTP `HEAD` only):** the official release asset
<https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt> reports `content-length: 5613764` and
`x-ms-blob-content-md5: JhR06RsV9e8UpjwhzmwMuw==`. The local file has the same size and the same MD5, so it is that asset.
GitHub does not publish a SHA-256 for it; the SHA-256 above was computed locally.

## How to obtain it

1. Get `yolo11n.pt` from Ultralytics' official distribution channels. The validated file is the v8.3.0 release asset
   <https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt> (size and MD5 match; see above). Related pages:
   - YOLO11 documentation: <https://docs.ultralytics.com/models/yolo11/>
   - Ultralytics asset releases: <https://github.com/ultralytics/assets/releases>
2. Read and follow the license terms that apply to the weights and to the `ultralytics` package:
   - <https://www.ultralytics.com/license>
   - <https://github.com/ultralytics/ultralytics/blob/main/LICENSE>
3. Place the file at `<workspace>/yolo11n.pt` and verify the checksum:

   ```bash
   cd <workspace>
   echo "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1  yolo11n.pt" | sha256sum -c -
   ```

   If the checksum differs, it's a different file. Detection results may then differ from the recorded evidence.

## Licensing note (unresolved; read before redistributing or deploying)

- The weights' embedded metadata, and the installed `ultralytics` packages (`ultralytics`, `ultralytics-thop`,
  `ultralytics-platform`), declare **AGPL-3.0**.
- `src/my_first_robot/my_first_robot/yolo_detector.py` imports `ultralytics` directly.
- **Keeping the weights out of this repository does not, by itself, settle every Ultralytics licensing question.** Running or
  distributing the detector uses AGPL-3.0 code, and Ultralytics offers separate commercial licensing. How AGPL-3.0 applies to
  this project as a whole hasn't been determined. See `docs/THIRD_PARTY_AND_LICENSING.md`.

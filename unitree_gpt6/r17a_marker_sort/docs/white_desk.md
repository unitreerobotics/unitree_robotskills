# White desk

English | [中文](white_desk.zh.md)

- The pen bodies are the same colour as the desk, so `perception/pens.py` finds pens by **pairing same-colour caps**: a cap of the same colour at each end, one pen length apart, white body between them, the ring about 30 mm from one end. On kraft paper it finds the white bar instead; both methods coexist.
- For a pen with one end behind a finger or out of frame, change the view (`look 0.35`) so both ends are in the image. Mark dark caps (brown, black, grey) by hand with `addpen`.
- **The box is easily dragged on a white desk**: one `reseat` can move it 3–8 mm and turn it 3–8°. Re-locate the box after every `reseat`.
- On a white desk "pen hangs straight" relies on a second criterion: the strip between the fingers shows the light-dark variation of a pen body, while an empty desk is flat. Blue pen bodies vary less; the threshold is 2.0.
- A pen can end up about 55 mm high even when lowering met no resistance (tip in the hole, body stuck, pen slid up in the fingers). `verify` after every placement.

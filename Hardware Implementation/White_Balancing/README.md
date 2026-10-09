# Underwater White Balance: The Full Two-Stage Pipeline

Based on Section III of *Color Balance and Fusion for Underwater Image Enhancement* (Ancuti et al., IEEE TIP, 2018).

The paper writes out only Eq. 4 and Eq. 5. The Gray World step is described in words ("the conventional Gray-World assumption"), so the formulas in Stage 2 are the standard textbook version, not equations from the paper.

## Step 0: Normalize the input

For an 8-bit image, with channels c ∈ {r, g, b}:

**I\_c(x) ← I\_c(x) / 255**, so every channel lies in \[0, 1\].

The paper states this requirement for Eq. 4: "each channel being in the interval \[0, 1\], after normalization by the upper limit of their dynamic range."

## Step 1: Compute the channel means

For an image with M × N pixels:

**Ī\_c = (1 / MN) · Σₓ I\_c(x)**, for c = r, g, b

This gives Ī\_r, Ī\_g, Ī\_b. Under the severe attenuation the paper targets, typically Ī\_g > Ī\_b > Ī\_r.

## Stage 1: Red (and optionally blue) compensation

### Step 2: Compensate the red channel (Eq. 4)

**I\_rc(x) = I\_r(x) + α · (Ī\_g − Ī\_r) · (1 − I\_r(x)) · I\_g(x)**, with α = 1.

Computed in this order:

1. **Global factor:** k\_r = α · (Ī\_g − Ī\_r), a single scalar for the whole image.
2. **Per-pixel weight:** w(x) = (1 − I\_r(x)) · I\_g(x).
3. **Correction:** Δ\_r(x) = k\_r · w(x).
4. **Result:** I\_rc(x) = I\_r(x) + Δ\_r(x).

**Bounds check.** If α = 1 and 0 ≤ Ī\_g − Ī\_r ≤ 1, then I\_rc ≤ I\_r + 1·(1 − I\_r)·1 = 1, so the output stays in \[0, 1\] without clipping. This is a consequence of the formula's design, not something the paper states. It assumes Ī\_g > Ī\_r. If red were stronger than green, the correction would become negative.

### Step 3 (optional): Compensate the blue channel (Eq. 5)

Used only for turbid or plankton-rich water, such as the Fig. 14 cases:

**I\_bc(x) = I\_b(x) + α · (Ī\_g − Ī\_b) · (1 − I\_b(x)) · I\_g(x)**, with α = 1.

It has the same structure as Eq. 4, with blue in place of red.

### Step 4: Assemble the compensated image

**I′ = ( I\_rc , I\_g , I\_b )**, or **( I\_rc , I\_g , I\_bc )** if blue compensation is used.

Green is never modified. This is the "Image Compensated" panel in Fig. 2.

## Stage 2: Gray World

### Step 5: Recompute the means on the compensated image

**Ī′\_c = (1 / MN) · Σₓ I′\_c(x)**, for c = r, g, b

These must be recomputed, because Ī′\_r is now larger than the original Ī\_r. That is the whole point of Stage 1.

### Step 6: Estimate the illuminant

First compute the target gray level, the average of the three channel means:

**μ = (Ī′\_r + Ī′\_g + Ī′\_b) / 3**

The Gray World assumption says the true scene averages to gray, so all three channel means should equal μ. The estimated illuminant strength per channel is then:

**e\_c = Ī′\_c / μ**

If e\_c > 1, channel c is over-represented (for example, the blue-green cast). If e\_c < 1, it is under-represented.

### Step 7: Divide out the illuminant

**I\_c^out(x) = I′\_c(x) / e\_c = I′\_c(x) · μ / Ī′\_c**, for c = r, g, b

The last form shows it as a per-channel gain g\_c = μ / Ī′\_c. The paper's description matches this: "dividing each color channel by its corresponding normalized light source intensity."

### Step 8: Clip

**I\_c^out(x) ← min(1, max(0, I\_c^out(x)))**

Gains above 1 can push bright pixels past 1. This clipping is an implementation detail not discussed in the paper.

**Check:** After Step 7, the mean of every channel equals μ, which is exactly the Gray World condition. Clipping in Step 8 can shift the means slightly.

## Illustrative numbers

These are made-up values to show the effect, not figures from the paper. Suppose Ī\_r = 0.1, Ī\_g = 0.5, Ī\_b = 0.4.

**Gray World alone (no Stage 1):**

- μ = (0.1 + 0.5 + 0.4) / 3 = 0.333
- Red gain = 0.333 / 0.1 = **3.33**, which amplifies noise and any real red heavily.

**With Stage 1 first:** The global factor is Ī\_g − Ī\_r = 0.4. Say the compensation lifts the red mean to Ī′\_r ≈ 0.3.

- μ = (0.3 + 0.5 + 0.4) / 3 = 0.4
- Red gain = 0.4 / 0.3 = **1.33**
- Green gain = 0.4 / 0.5 = 0.8
- Blue gain = 0.4 / 0.4 = 1.0

The red gain drops from 3.33 to 1.33, so the red artifacts largely go away. The jump in Ī′\_r depends on the actual pixel distribution, since the correction is weighted per pixel by (1 − I\_r)·I\_g.

## Full pipeline summary

| # | Operation | Formula |
| --- | --- | --- |
| 0 | Normalize | I\_c ← I\_c / 255 |
| 1 | Means | Ī\_c = (1/MN) Σ I\_c(x) |
| 2 | Red compensation | I\_rc = I\_r + α(Ī\_g − Ī\_r)(1 − I\_r) I\_g |
| 3 | Blue compensation (optional) | I\_bc = I\_b + α(Ī\_g − Ī\_b)(1 − I\_b) I\_g |
| 4 | Assemble | I′ = (I\_rc, I\_g, I\_b or I\_bc) |
| 5 | Recompute means | Ī′\_c |
| 6 | Gray level | μ = (Ī′\_r + Ī′\_g + Ī′\_b)/3 |
| 7 | Gain per channel | I\_c^out = I′\_c · μ / Ī′\_c |
| 8 | Clip | to \[0, 1\] |

The output I^out is the white-balanced image that feeds both fusion inputs in Section IV: the gamma-corrected version and the normalized unsharp-masked version.

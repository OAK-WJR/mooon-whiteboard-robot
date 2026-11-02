# Cost review of the five finished top boards

**Only counts the five top PCBs, all components on them, and soldered finished boards.** Total target ¥30, cap ¥50, CNY preferred; the same figure in USD is only a possible fallback. The base board, motors, battery, charger and external accessories are not counted, and no more of them are being bought.

When the three supplier pages were last refreshed, they returned access verification content, and no usable new prices or stock were obtained; the amounts below are kept as historical references.

**There is currently no complete finished-board quote, and the budget has not passed.** The current drawing has 70 components per board; 63 references have store part numbers, and 7 are still to be chosen; the full input connector, protection and the undecided USB/ADC circuits may still change the list. All items with missing prices stay unknown. [Current list](TOP_COMPONENTS.md).

## China-site CNY reference: only the three chips chosen for the top

On 2026-09-09, the public pages of JLC's China site were read through a web-access tool, giving the table below. The pages show **pre-order reference prices**, and all three show stock 0; the pages the tool returned were captured about one month, one month and two months earlier, so **this is not a five-board order quote that can be placed today**. Quantities count only the current top board's actual placement use, with no spares or loss.

| Top board reference / part number | Pre-order reference unit price CNY | Use for five boards | Reference subtotal CNY |
| --- | ---: | ---: | ---: |
| U3: ESP32-C6-MINI-1-N4 / C5736265 | 17.21 | 5 | 86.05 |
| U1: CH343G / C2844153 | 5.85 | 5 | 29.25 |
| U5: TPS62933DRLR / C3200405 | 1.40 | 5 | 7.00 |
| **Only these three chips** | | **15** | **122.30** |

Sources: [China-site MINI page](https://www.jlc-smt.com/lcsc/detail/C5736265.html), [China-site CH343G page](https://www.jlc-smt.com/lcsc/detail/C2844153.html), [China-site TPS62933 page](https://www.jlc-smt.com/lcsc/detail/C3200405.html). All three are mounted on the top PCB; this does not include motors, base board, battery or charger.

At these public reference prices, **five MINI controllers alone exceed the ¥50 cap for five complete finished top boards**. ¥122.30 still lacks the other 67 components per board already drawn, the necessary parts not yet drawn, the PCB and all assembly fees; it is not a single finished-board price, nor a five-board total quote. China-site assembly eligibility and fees must be verified with a China-site order; the global site's Standard Only or USD fees below are not applied. For the item-by-item record and its binding to the current drawings, see the [CNY reference record](top_domestic_price_reference_2026-09-09.json).

## Cost limits of the current MINI layout

To evaluate a layout within the original triangular board outline, the ESP32-C6-MINI-1-N4 / C5736265 was tried. The JLCPCB global site page checked on 2026-09-09 lists it as an **extended part, Standard Only, X-ray inspection required**. So the economic engineering fee and loading fee of the original WROOM plan cannot be applied to the new drawing; China-site eligibility must be confirmed separately. This swap is only a space/wiring review, **not the low-cost final design**. [JLC exact part page](https://jlcpcb.com/partdetail/ESPRESSIFINC-ESP32_C6_MINI_1N4/C5736265).

The LCSC page's 1+ unit price this time is **$3.9252**, so five modules alone are about **$19.63**. This is the component retail price, not the JLC assembly billing price. [LCSC](https://www.lcsc.com/product-detail/C5736265.html).

The global site reference only counts the items below. The MINI price and Standard fees are from this check; the other three chips use retail references saved earlier the same day, so this is a partial cost calculation from different times and cannot be used as an order quote:

| Item needed for five boards | Quantity | USD subtotal |
| --- | ---: | ---: |
| ESP32-C6-MINI-1-N4 | 5 | 19.6260 |
| CH343G | 5 | 6.2115 |
| TPS563252DRLR | 5 | 0.9650 |
| TPS62933DRLR | 5 | 2.8955 |
| Standard single-side engineering fee | One batch | 25.5600 |
| Standard single-side stencil | One batch | 8.2100 |
| **Only the items above, still incomplete** | | **63.4680 ≈ $63.47** |

This number already exceeds the possible $50 fallback cap, **and does not yet include the PCB, the other on-board components, all loading fees, solder joints, X-ray, manual through-hole insertion/fixtures, handling fees, loss, tax and shipping**. Even counting only the top, the current plan is not shown to fit the budget. Basis for Standard pricing: [JLCPCB public fee table](https://jlcpcb.com/help/article/pcb-assembly-price). Mixing public retail prices with processing fees is only for screening; the final order may differ.

## CNY range

The China-site rules saved earlier list an economic engineering fee of ¥50 and a free stencil; this time the fee page failed to load, and no new full quote was obtained. With a ¥50 engineering fee and no discount, the engineering fee alone uses up the cap for the whole batch, and the PCB, components and soldering, which all cost money, still have to be added. Discounts, China-site MINI assembly eligibility and the actual fees are not verified and cannot be assumed to be free. [Source of the saved record](https://www.jlc.com/portal/server_guide_51484.html).

The old five-module price of **¥125.90** is for the WROOM/C6034714, not the current MINI's CNY price, and has been moved into a history field. The USD Standard fees were not converted or passed off as a China-site quote.

## Historical records and conditions for a later final design

The old WROOM stage's $52.695, $58.835 and $64.975 are all calculations under economic conditions that cover only some of the top items, without motors, charger, base board or battery; they are not the current MINI finished-board price. For the historical details, see the [pre-migration record](controller_core/archive/cost_before_MINI.md) and the [structured cost record](cost_review_2026-09-09.json).

A later low-cost final design must meet the triangular space, the 19 contacts, the electrical requirements and the actual assembly eligibility of that exact part number all at once; comparing chip unit prices alone is not enough. The final delivery must include all top components and soldering; buying the controller separately, soldering by hand, or leaving out on-board protection that has not been designed cannot be assumed by default. There is currently no purchase and no manufacturing release.

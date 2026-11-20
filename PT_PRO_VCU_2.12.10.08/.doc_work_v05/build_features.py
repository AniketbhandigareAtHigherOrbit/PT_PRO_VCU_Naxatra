from pathlib import Path
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.opc.constants import RELATIONSHIP_TYPE as RT
import json

ROOT = Path(__file__).resolve().parent.parent
ITEMS = [
('Automatic working depth control',
 'Let the operator select a cultivation depth and have the machine adjust its depth wheel or attachment position to maintain it. For example, it could compensate when changing ground conditions make the rotor dig too deeply, instead of requiring repeated manual adjustment. This needs a suitable ground-referenced depth sensor, a powered adjustment mechanism, and a mechanically compatible attachment.',
 'John Deere TruSet depth control', 'https://www.deere.ca/en/tillage/truset/'),
('Travel that follows the operator\'s pace',
 'Use a sensing handle to adjust travel speed to the operator\'s walking pace while the safety lever remains held. For example, easing pressure on the handle would gently reduce travel speed without repeatedly changing the throttle knob. A spring-loaded handle with position or force sensing is required; releasing the safety lever would still command a stop.',
 'Toro Personal Pace electric mower', 'https://www.toro.com/en/product/21469'),
('Remaining working time display',
 'Show an estimated range of working minutes remaining using available battery energy and recent power demand, rather than displaying SOC alone. For example, the operator could see that heavy cultivation is likely to exhaust the batteries sooner than travelling between plots and plan a battery change. This needs validated BMS measurements, usable-capacity information, and display support; a better battery gauge may be needed if those measurements are inadequate.',
 'Texas Instruments battery runtime estimation', 'https://www.ti.com/lit/wp/slpy002/slpy002.pdf'),
('Automatic attachment recognition',
 'Recognize an approved attached tool and select its permitted rotor speeds, directions, and operating controls. For example, changing from a compatible tiller to a brush attachment would load the appropriate tool profile without changing firmware, while an unknown tool would remain disabled. This requires compatible mechanical and electrical connections plus a coded connector, identification tag, or small CAN module; recognition would not start the tool automatically.',
 'Bobcat attachment recognition', 'https://www.bobcat.com/eu/en/attachments/rotary-telehandler-attachments?page=2'),
('Operator commanded row end sequence',
 'Give the operator one control that stops the rotor and selects a limited manoeuvring speed at the end of a row. After the operator turns the machine, a separate deliberate resume command would restore the selected working settings, reducing repeated adjustments on short plots. This requires an additional button or compatible display control; automatic attachment lifting would also need a powered lift and position sensing.',
 'John Deere integrated implement and headland control', 'https://www.deere.com/assets/pdfs/region-1/products/tractors/7R_Brochure.pdf'),
('Automatic parking hold without continuous motor power',
 'Add a mechanical holding brake that engages after a controlled stop and releases only when the drive is ready to move. For example, a stopped machine could remain parked within its rated slope limit without relying on a continuous low-RPM motor command. This needs a correctly sized spring-applied brake, release driver, and brake-state feedback, with coordinated stopping and release control.',
 'BCS two-wheel equipment with automatic parking hold', 'https://bcsagri.com/fr-fr/product/660-hy-ws-powersafe/'),
('Ground slip detection and traction assistance',
 'Compare wheel rotation with independently measured ground movement to recognize when the wheels are spinning without making useful progress. In a muddy patch, the display could warn the operator and a bounded traction-assist mode could reduce the drive request instead of allowing continued wheel spin. This needs a ground-speed sensor suitable for very slow field operation; motor-controller support would determine how precisely drive effort can be limited.',
 'DICKEY-john independent radar ground-speed sensing', 'https://dickey-john.com/products/sensors/speed-sensors/radar-iii-speed-sensor/'),
('Battery condition history by serial number',
 'Build a service history for each battery using delivered energy, cell-voltage spread, voltage sag, and temperature under comparable loads. For example, it could flag a pack that repeatedly sags more than its partner for inspection before the next working day, without claiming a definite battery fault. This can build on the existing BMS data and serial numbers, but needs validated measurements and persistent storage on the VCU or gateway.',
 'Husqvarna battery health and usage monitoring', 'https://www.husqvarna.com/uk/services/fleet-services/'),
('Automatic fault event recording',
 'Preserve a short recording from before and after a drive fault, rotor stop, or abnormal reversal, including commands, actual RPM, currents, and operator inputs. For example, a technician could inspect what both wheels did during a brief sideways jerk instead of relying only on an error code or a later snapshot. This needs sufficient recording memory or a CAN logger, and faster event data than the normal periodic telemetry where necessary.',
 'CSS Electronics CAN black-box logging', 'https://www.csselectronics.com/pages/black-box-can-bus-logger'),
('Component specific service reminders',
 'Track actual rotor-working hours and wheel-drive hours separately, retain them across power cycles, and show the next scheduled service. For example, gearbox service could become due after its specified operating hours even if the machine spent several calendar days unused. This needs persistent counters and a service-reset control on the display or service tool, with intervals set for the actual components.',
 'STIHL operating-time-based maintenance', 'https://www.stihlusa.com/en/support-events/faq/stihl-connected'),
('Rotor vibration warning',
 'Monitor vibration near the rotor or gearbox and flag persistent changes at comparable speeds and working conditions. For example, an unusual pattern after striking an obstruction could prompt inspection for a damaged tine, loose mounting, or bearing problem, without presenting the warning as a confirmed diagnosis. This requires a suitably mounted vibration sensor and sufficient signal-processing capability, potentially in a dedicated sensor module.',
 'Analog Devices vibration condition monitoring', 'https://www.analog.com/en/resources/technical-articles/2019/06/27/03/41/choose-the-right-accelerometer-for-predictive-maintenance.html'),
('Field job records and coverage maps',
 'Let the operator start a named job and save its worked area, active working time, and measured battery energy use, excluding travel with the tool disengaged. For example, a contractor could review coverage and overlapping passes after finishing a plot. This needs position accuracy appropriate to the narrow working width, a reliable tool-in-work signal, and a display or gateway; an upgraded GNSS receiver may be necessary.',
 'John Deere work-data and coverage mapping', 'https://displaysimulator.deere.com/onscreen_help/4640/current/en/mapping/map_work_data.htm'),
('Remote manoeuvring with the rotor disabled',
 'Provide an optional handheld remote for low-speed positioning while the operator stays beside the machine with a clear view. For example, the operator could guide it through a narrow storage entrance without standing directly behind the handle. This requires an industrial radio receiver, a held-to-run control, an emergency stop, local mode selection, and a defined stop on link loss; the rotor would remain disabled in this mode.',
 'Agria remotely controlled field equipment', 'https://agria.de/de/produkte/ferngesteuerte-maehtechnik/agria-9500-70'),
('Authorized operator start access',
 'Require an approved operator tag before enabling a new operating session, while keeping normal stopping controls available at all times. For example, a shared or rented machine could be assigned to specific operators and record who used it, without stopping a moving machine merely because a tag leaves range. This needs an identification reader or compatible gateway; Teltonika support depends on the exact device and its interfaces.',
 'Teltonika operator authorization and immobilizer', 'https://wiki.teltonika-gps.com/view/Immobilizer_configuration_explained'),
('Scheduled battery readiness and charging coordination',
 'Allow a planned departure time and coordinate compatible chargers so the required battery pair is ready, with an alert if either pack fails to charge. For example, a depot could stage several machines\' charging within its electrical supply limit instead of starting every charger together. This needs controllable approved chargers and battery-status communication, with scheduling in a gateway or charging station; off-machine packs would need a connected dock.',
 'STIHL charging management', 'https://www.stihl.co.uk/en/stihl-technology/battery-technology/charging-management'),
]

def link(paragraph, label, url):
    element = OxmlElement('w:hyperlink')
    element.set(qn('r:id'), paragraph.part.relate_to(url, RT.HYPERLINK, is_external=True))
    run = OxmlElement('w:r')
    props = OxmlElement('w:rPr')
    color = OxmlElement('w:color'); color.set(qn('w:val'), '245579'); props.append(color)
    size = OxmlElement('w:sz'); size.set(qn('w:val'), '19'); props.append(size)
    under = OxmlElement('w:u'); under.set(qn('w:val'), 'single'); props.append(under)
    run.append(props)
    text = OxmlElement('w:t'); text.text = label; run.append(text)
    element.append(run); paragraph._p.append(element)

doc = Document()
sec = doc.sections[0]
sec.page_width = Inches(8.27); sec.page_height = Inches(11.69)
sec.top_margin = sec.bottom_margin = Inches(.65)
sec.left_margin = sec.right_margin = Inches(.75)
normal = doc.styles['Normal']
normal.font.name = 'Calibri'; normal.font.size = Pt(11)
normal.font.color.rgb = RGBColor(0,0,0)
normal.paragraph_format.line_spacing = 1.08
normal.paragraph_format.space_after = Pt(5)
for name in ['Title','Heading 1']:
    doc.styles[name].font.name = 'Calibri'
    doc.styles[name].font.color.rgb = RGBColor(0,0,0)
doc.styles['Title'].font.size = Pt(22)
doc.styles['Title'].paragraph_format.space_after = Pt(16)
doc.core_properties.title = 'Proposed New Features for VCU Version 05'
doc.core_properties.subject = 'Fifteen proposed features for the next VCU release'
doc.core_properties.author = ''
doc.add_paragraph('Proposed New Features for VCU Version 05', 'Title')

# Explicit page groups keep each complete proposal together and the list readable.
for i,(name,body,label,url) in enumerate(ITEMS,1):
    if i in (6,11):
        doc.add_page_break()
    p=doc.add_paragraph()
    p.paragraph_format.keep_with_next=True
    p.paragraph_format.space_before=Pt(8 if i not in (1,6,11) else 0)
    p.paragraph_format.space_after=Pt(4)
    r=p.add_run(f'{i}. {name}'); r.bold=True; r.font.size=Pt(12)
    p=doc.add_paragraph(body)
    p.paragraph_format.keep_with_next=True
    p.paragraph_format.keep_together=True
    p=doc.add_paragraph()
    p.paragraph_format.space_after=Pt(7)
    r=p.add_run('Related reference: '); r.font.size=Pt(9.5)
    link(p,label,url)

output=ROOT/'Proposed_New_Features_for_VCU_Version_05.docx'
doc.save(output)
(ROOT/'.doc_work_v05'/'features.json').write_text(json.dumps(ITEMS,ensure_ascii=False,indent=2),encoding='utf-8')
print(output)
print('Features:', len(ITEMS))

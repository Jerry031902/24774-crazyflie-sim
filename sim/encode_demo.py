#!/usr/bin/env python3
"""Compose actual viewport captures into a 1080p, H.264 presentation video.

Host: python3 sim/encode_demo.py
Requires Pillow and ffmpeg. Uses the existing presentation's white/red style;
no slide deck is created. All video imagery comes from record_demo.py captures.
"""
import argparse
import csv
import json
import shutil
import subprocess
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

SIM_DIR = Path(__file__).resolve().parent
SIZE = (1920, 1080)
RESAMPLE = getattr(Image, 'Resampling', Image).LANCZOS
RED = '#BB0A30'
INK = '#191919'
GRAY = '#555555'
LIGHT = '#E3E3E3'
FONT_DIR = Path('/usr/share/fonts/truetype/liberation2')


def font(size, bold=False):
    name = 'LiberationSans-Bold.ttf' if bold else 'LiberationSans-Regular.ttf'
    path = FONT_DIR / name
    if not path.exists():
        path = Path('/usr/share/fonts/truetype/liberation') / name
    return ImageFont.truetype(str(path), size)


FONTS = {s: font(s) for s in (20, 22, 24, 26, 28, 30, 32, 40, 46)}
BOLD = {s: font(s, True) for s in (20, 22, 24, 26, 28, 30, 32, 40, 46)}


def compose(raw, row, noise):
    """Main fixed camera plus a clearly labelled crop of the same frame."""
    canvas = Image.new('RGB', SIZE, 'white')
    d = ImageDraw.Draw(canvas)
    d.rectangle((0, 0, 1920, 8), fill=RED)
    d.text((40, 28), 'Crazyflie | Stock EKF + Firmware PID', font=BOLD[46], fill=INK)
    d.text((42, 88), 'Simulated IMU / Flow / ToF  >  Stock EKF  >  controllerPid  >  powerDistribution  >  PhysX',
           font=FONTS[24], fill=GRAY)
    # Keep the entire 16:9 viewport. Never crop away the takeoff or target region.
    main_box = (40, 150, 1440, 938)
    canvas.paste(raw.resize((1400, 788), RESAMPLE), main_box[:2])
    d.rectangle(main_box, outline=LIGHT, width=2)
    # Close-up is a pixel crop from the SAME renderer frame, centered on the
    # projected body COM. It is not a second simulation or a scaled USD asset.
    cx, cy = float(row['screen_x_px']), float(row['screen_y_px'])
    crop_w, crop_h = 240, 168
    left = max(0, min(raw.width-crop_w, round(cx-crop_w/2)))
    top = max(0, min(raw.height-crop_h, round(cy-crop_h/2)))
    crop = raw.crop((left, top, left+crop_w, top+crop_h))
    d.text((1480, 150), 'DRONE CLOSE-UP', font=BOLD[24], fill=RED)
    canvas.paste(crop.resize((400, 280), RESAMPLE), (1480, 190))
    d.rectangle((1480, 190, 1880, 470), outline=LIGHT, width=2)
    d.text((1480, 480), 'Same frame, enlarged view', font=FONTS[20], fill=GRAY)
    phase = {'takeoff': '01  TAKEOFF / SETTLE', 'hover': '02  HOVER', 'move': '03  MOVE + HOLD'}[row['phase']]
    d.text((1480, 530), phase, font=BOLD[28], fill=RED)
    t = float(row['time_s'])
    d.text((1480, 576), f'{t:04.1f} s / 30.0 s', font=BOLD[40], fill=INK)
    goal_x = float(row['target_x_m'])
    d.text((1480, 636), f'Target: ({goal_x:.1f}, 0.0, 1.0) m', font=FONTS[24], fill=GRAY)
    d.line((1480, 685, 1880, 685), fill=LIGHT, width=2)
    d.text((1480, 708), 'TRUE POSITION', font=BOLD[22], fill=RED)
    for i, axis in enumerate('xyz'):
        value = float(row[f'{axis}_m'])
        if abs(value) < .0005:
            value = 0.0
        d.text((1480, 750+i*48), axis.upper(), font=BOLD[28], fill=GRAY)
        d.text((1550, 750+i*48), f'{value: .3f} m', font=FONTS[30], fill=INK)
    # A small unobtrusive locator makes the true centimeter-scale drone visible
    # in the wide scene. Label and open corner brackets don't cover the vehicle.
    px, py = 40 + cx*1400/1920, 150 + cy*788/1080
    if 70 < px < 1400 and 185 < py < 900:
        radius = 36
        color = '#FFFFFF'
        for sx in (-1, 1):
            for sy in (-1, 1):
                x,y=px+sx*radius,py+sy*radius
                d.line((x,y,x-sx*10,y),fill=color,width=2)
                d.line((x,y,x,y-sy*10),fill=color,width=2)
    # Phase strip uses the same 10/10/10 second schedule as flight_demo.py.
    labels = ['Takeoff to Z = 1.0 m', 'Hold for 10 s', 'Move to X = 0.5 m']
    for i, label in enumerate(labels):
        x0 = 40 + i*620
        d.rectangle((x0, 966, x0+600, 972), fill=LIGHT)
        fill = min(1.0, max(0.0, (t-i*10)/10))
        if fill > 0:
            d.rectangle((x0, 966, x0+600*fill, 972), fill=RED)
        d.text((x0, 984), label, font=FONTS[24], fill=INK)
    d.text((40, 1038), f'Isaac Sim 5.1  |  ekf_closed_loop  |  Sensor noise {noise.upper()}  |  1x simulation time',
           font=FONTS[20], fill=GRAY)
    d.text((1180, 1038), 'Baseline demo  |  Mass / inertia not yet calibrated', font=FONTS[20], fill=GRAY)
    return canvas


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=SIM_DIR.parent/'build/demo_capture')
    parser.add_argument('--output',type=Path,default=SIM_DIR/'demos')
    parser.add_argument('--preview',action='store_true',help='Create one composited PNG only')
    args=parser.parse_args()
    meta=json.loads((args.input/'capture.json').read_text())
    with (args.input/'frames.csv').open() as f:
        rows=list(csv.DictReader(f))
    if len(rows)!=meta['frames']:
        raise ValueError('Incomplete recording: frame metadata count mismatch')
    for i in range(len(rows)):
        if not (args.input/'frames'/f'{i:05d}.png').is_file():
            raise ValueError(f'Missing frame {i}')
    args.output.mkdir(parents=True,exist_ok=True)
    basename=f'crazyflie_ekf_baseline_noise_{meta["noise"]}'
    poster=args.output/f'{basename}.png'
    selected=min(15*meta['fps'],len(rows)-1)
    with Image.open(args.input/'frames'/f'{selected:05d}.png') as raw:
        compose(raw,rows[selected],meta['noise']).save(poster)
    if args.preview:
        print(poster)
        return
    output=args.output/f'{basename}.mp4'
    if output.exists():
        raise FileExistsError(output)
    ffmpeg=shutil.which('ffmpeg')
    if ffmpeg is None:
        raise RuntimeError('ffmpeg must be on PATH')
    command=[ffmpeg,'-hide_banner','-loglevel','warning','-f','rawvideo','-pixel_format','rgb24',
             '-video_size','1920x1080','-framerate',str(meta['fps']),'-i','pipe:0','-an',
             '-c:v','libx264','-preset','medium','-crf','19','-pix_fmt','yuv420p',
             '-movflags','+faststart','-metadata','title=Crazyflie stock EKF and firmware PID baseline',
             str(output)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE)
    try:
        for i,row in enumerate(rows):
            with Image.open(args.input/'frames'/f'{i:05d}.png') as raw:
                if raw.size!=tuple(meta['resolution']):
                    raise ValueError(f'Wrong source resolution at frame {i}')
                process.stdin.write(compose(raw,row,meta['noise']).tobytes())
            if i%150==0:
                print(f'ENCODE {i}/{len(rows)}',flush=True)
        process.stdin.close()
        if process.wait()!=0:
            raise RuntimeError('ffmpeg failed')
    except BaseException:
        process.kill()
        process.wait()
        raise
    # Also provide a clean viewport-only MP4 for slides with their own titles.
    clean=args.output/f'{basename}_clean.mp4'
    if clean.exists():
        raise FileExistsError(clean)
    subprocess.run([ffmpeg,'-hide_banner','-loglevel','warning','-framerate',str(meta['fps']),
                    '-i',str(args.input/'frames'/'%05d.png'),'-frames:v',str(len(rows)),
                    '-an','-c:v','libx264','-preset','medium','-crf','19',
                    '-pix_fmt','yuv420p','-movflags','+faststart',str(clean)],check=True)
    shutil.copyfile(args.input/'frames.csv',args.output/f'{basename}.frames.csv')
    meta['video_file']=output.name
    meta['clean_video_file']=clean.name
    meta['clean_video_bytes']=clean.stat().st_size
    meta['video_bytes']=output.stat().st_size
    meta['encoding']='H.264, 1920x1080, 30 fps, yuv420p, faststart, no audio'
    meta['presentation']='White background, Arial-compatible Liberation Sans, CMU red #BB0A30.'
    (args.output/f'{basename}.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(f'SAVED {output} ({output.stat().st_size/1024/1024:.1f} MiB)',flush=True)


if __name__=='__main__':
    main()

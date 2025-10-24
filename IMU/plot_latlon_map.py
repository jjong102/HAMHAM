# plot_multi_latlon_map_hashedcolor.py
import os, argparse, glob, math, folium, hashlib

def load_latlons_csv(path):
    pts=[]
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            s=line.strip()
            if not s:
                continue
            s = s.replace(',', ' ').replace('\t', ' ')
            parts = [p for p in s.split() if p]
            if len(parts) < 2:
                continue
            try:
                lat = float(parts[0]); lon = float(parts[1])
                if not (math.isfinite(lat) and math.isfinite(lon)):
                    continue
            except Exception:
                continue
            pts.append((lat, lon))
    return pts

def path_to_color_hex(path: str) -> str:
    """
    경로 문자열을 해시→HSL로 매핑→HEX 변환.
    """
    h = int(hashlib.md5(path.encode('utf-8')).hexdigest(), 16)
    H = (h % 360) / 360.0
    S = 0.65
    L = 0.50
    def hue2rgb(p, q, t):
        if t < 0: t += 1
        if t > 1: t -= 1
        if t < 1/6: return p + (q - p) * 6 * t
        if t < 1/2: return q
        if t < 2/3: return p + (q - p) * (2/3 - t) * 6
        return p
    q = L * (1 + S) if L < 0.5 else L + S - L * S
    p = 2 * L - q
    r = hue2rgb(p, q, H + 1/3)
    g = hue2rgb(p, q, H)
    b = hue2rgb(p, q, H - 1/3)
    return '#{0:02x}{1:02x}{2:02x}'.format(int(r*255), int(g*255), int(b*255))

def main():
    ap = argparse.ArgumentParser()
    # ap.add_argument('--pattern', default='~/IMU/second/gps_latlon*.csv',
    ap.add_argument('--pattern', default='~/IMU/second/gps_latlon*.csv',
                    help='불러올 CSV 글롭 패턴 (예: imu/second/latlon*.csv)')
    ap.add_argument('--outfile', default='~/IMU/gps_map_multi.html')
    ap.add_argument('--zoom', type=int, default=18, help='초기 줌 레벨')
    ap.add_argument('--max-zoom', type=int, default=22, help='최대 줌 레벨')
    ap.add_argument('--no-fit', action='store_true',
                    help='전체 영역 맞춤(fit_bounds) 비활성화')
    ap.add_argument('--tiles', default='OpenStreetMap',
                    help='타일 이름 또는 URL (예: Stamen Terrain, CartoDB positron 등)')
    args = ap.parse_args()

    pattern = os.path.expanduser(os.path.expandvars(args.pattern))
    outfile = os.path.expanduser(os.path.expandvars(args.outfile))

    files = sorted(glob.glob(pattern))
    if not files:
        raise SystemExit(f'파일 없음: {pattern}')

    all_pts = []
    tracks = []
    for fp in files:
        pts = load_latlons_csv(fp)
        if pts:
            tracks.append((fp, pts))
            all_pts.extend(pts)

    if not tracks:
        raise SystemExit('CSV에서 유효한 (lat, lon)을 찾지 못했습니다.')

    m = folium.Map(location=all_pts[0], zoom_start=args.zoom,
                   max_zoom=args.max_zoom, control_scale=True, tiles=args.tiles)

    print('=== File → Color ===')
    for fp, _ in tracks:
        print(f'{fp} -> {path_to_color_hex(fp)}')

    for idx, (fp, pts) in enumerate(tracks, start=1):
        name = os.path.basename(fp)
        color = path_to_color_hex(fp)
        layer = folium.FeatureGroup(name=f'{idx:02d} - {name}', show=True)

        folium.Marker(pts[0], tooltip=f"[{idx:02d}] Start: {name}").add_to(layer)
        if len(pts) > 1:
            folium.Marker(pts[-1], tooltip=f"[{idx:02d}] End: {name}").add_to(layer)
            folium.PolyLine(pts, weight=3, color=color).add_to(layer)

        for i, (lat, lon) in enumerate(pts, start=1):
            folium.CircleMarker((lat, lon), radius=2, color=color, fill=True,
                                fill_opacity=0.7,
                                tooltip=f"[{idx:02d}] #{i} {lat:.7f},{lon:.7f}").add_to(layer)

        layer.add_to(m)

    if not args.no_fit:
        m.fit_bounds(all_pts)

    folium.LayerControl(collapsed=False).add_to(m)

    os.makedirs(os.path.dirname(outfile) or ".", exist_ok=True)
    m.save(outfile)
    print(f"Saved map -> {outfile}")

if __name__ == "__main__":
    main()

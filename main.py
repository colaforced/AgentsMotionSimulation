import argparse

from formation import Simulation


def main():
    parser = argparse.ArgumentParser(description='基于自组织的多无人机均匀多边形编队仿真')
    parser.add_argument('-n', type=int, default=10, help='智能体数量（默认 10）')
    parser.add_argument('-m', type=int, default=5, help='多边形边数（默认 5）')
    parser.add_argument('-r', type=float, default=30, help='多边形外接圆半径（默认 30）')
    parser.add_argument('--height', type=float, default=200, help='编队高度（默认 200）')
    parser.add_argument('--seed', type=int, help='随机种子，固定后可复现')
    parser.add_argument('--headless', action='store_true', help='不画图，跑完后打印结果')
    parser.add_argument('--save', metavar='FILE.gif', help='保存为 GIF，不弹窗')
    parser.add_argument('--rounds-per-frame', type=int, default=1, help='每帧推进的轮数，调大可加速动画')
    parser.add_argument('--fps', type=int, default=30, help='保存 GIF 的帧率（默认 30）')
    args = parser.parse_args()

    sim = Simulation(n=args.n, m=args.m, r=args.r, h=args.height, seed=args.seed)
    if args.headless:
        ok = sim.run()
        print(f"{'编队完成' if ok else '未完成'}，共 {sim.rounds} 轮")
        return

    from formation.visualize import animate
    animate(sim, rounds_per_frame=args.rounds_per_frame, save=args.save, fps=args.fps)


if __name__ == '__main__':
    main()

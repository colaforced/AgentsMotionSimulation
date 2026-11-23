import matplotlib.pyplot as plt
from matplotlib import animation


def animate(sim, rounds_per_frame=1, interval=20, save=None, fps=30):
    """3D 动画展示仿真过程。save 为 .gif 路径时保存成文件而不弹窗。"""
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(projection='3d')
    pgn = sim.pgn
    top = pgn.h + sim.params.stagger_span / 2 + 25
    ax.set(xlim=(0, sim.area), ylim=(0, sim.area), zlim=(0, top), xlabel='X', ylabel='Y', zlabel='Z')

    vx = list(pgn.vertexs[:, 0]) + [pgn.vertexs[0, 0]]
    vy = list(pgn.vertexs[:, 1]) + [pgn.vertexs[0, 1]]
    ax.plot(vx, vy, [pgn.h] * len(vx), color='black', linestyle=':')

    p = sim.positions()
    sc = ax.scatter(p[:, 0], p[:, 1], p[:, 2], color='r', alpha=0.7, depthshade=False)
    title = ax.set_title('')

    def frames():
        yield
        while not sim.done:
            for _ in range(rounds_per_frame):
                sim.step()
                if sim.done:
                    break
            yield
        if save:  # GIF 循环播放，结尾停留 2 秒便于看清最终编队
            for _ in range(fps * 2):
                yield

    def draw(_):
        p = sim.positions()
        sc.set_offsets(p[:, :2])
        sc.set_3d_properties(p[:, 2], 'z')
        title.set_text(f'round {sim.rounds}  |  {sim.phase.name}')
        return sc, title

    ani = animation.FuncAnimation(fig, draw, frames=frames, interval=interval,
                                  cache_frame_data=False, save_count=10 ** 6)
    if save:
        ani.save(save, writer=animation.PillowWriter(fps=fps))
        plt.close(fig)
    else:
        plt.show()

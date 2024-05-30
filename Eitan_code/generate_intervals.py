import pickle
from time import sleep, time
from Eitan_code.CameraController import CameraWrapper
from Eitan_code.Robots import TaskRobot, AssistanceRobot

assistance_robot = AssistanceRobot()
task_robot = TaskRobot()


def save(filename, data):
    with open(filename, 'wb') as f:
        pickle.dump(data, f)


def compute_intervals():
    intervals = []
    camera = CameraWrapper()
    state = camera.is_visible()
    interval_start = 0

    task_robot.execute_task()
    print('starting')
    start_time = time()
    while task_robot.is_running():
        new_state = camera.is_visible(show=True)
        if new_state != state:
            if state is True:  # if stopped seeing task
                intervals.append((interval_start, time() - start_time))
            if state is False:  # if started seeing task
                interval_start = time() - start_time

            state = new_state

    total_time = time() - start_time
    print(total_time)
    if state is True:
        intervals.append((interval_start, total_time))

    # normalize intervals
    intervals = [(start / total_time, end / total_time) for start, end in intervals]
    return intervals


def generate_intervals(graph, configs, root=0):
    """
    Generates intervals for all nodes in the graph
    :param graph: adjacency list representation of the graph
    :param configs: configurations for all nodes
    :param root: root node of the graph
    """
    intervals = {}

    def aux(current_node):
        # move to the current node
        assistance_robot.move(configs[current_node])
        # compute intervals
        intervals[current_node] = compute_intervals()

        save('intervals.pkl', intervals)

        # generate intervals for all neighbors
        for v, _ in graph[current_node]:
            if v not in intervals:
                aux(v)

    aux(root)
    return intervals


def generate_from_list(samples):
    task_robot.move_home()
    Intervals = {(0, -1.5707963267948966, 0, -1.5707963267948966, 0, 0): [], (-1.9084609190570276, -3.096548696557516, -0.006988062057644129, -0.045498446827270556, -0.868300739918844, -0.00022489229311162262): [(0.07296929526623469, 0.21237443930901712)], (-1.7499454657184046, -2.661220212975973, 0.003704849873678029, -0.4731615346721192, -0.5209363142596644, -0.0002592245685022476): [(0.0, 0.08457740901188328), (0.16548314634283848, 0.45622793297446534), (0.8932623979837004, 1.0)], (-1.7122295538531702, -2.518874307672018, -0.0005792640149593353, 0.45814983427014155, -0.13690025011171514, -1.0553653875934046): [(0.0, 0.09300214386651827), (0.1613572539904501, 0.35299946643694335), (0.37491019060078473, 0.3772307628692813), (0.40520144354179244, 0.4073087209694751), (0.41378274940681875, 0.4238969020555514), (0.42795922788724833, 0.430366706778441), (0.43404266956305737, 0.43601746150389686), (0.4402481880908684, 0.4439530759238859), (0.45012461299805445, 0.46611761190921813), (0.5421565132287257, 0.6597922856118006), (0.7463209707767771, 0.7829835956244487), (0.891709549192219, 0.9018567903432894), (0.911916182616299, 0.914247766124812), (0.93037348608666, 0.9327530205324183), (0.961223523850009, 0.9633346196032098), (0.9676326188406293, 1.0)], (-1.8215530554400843, -2.5260197124876917, -0.00019462875206954777, 0.4559323030659179, -0.6241739431964319, -1.0540202299701136): [(0.0, 0.03393927629982044), (0.07947498101440154, 0.23693853655576075), (0.42635397214300097, 0.4505518006725414), (0.45378372662336924, 0.46881220320187506)], (-2.069029156361715, -3.0399538479247035, -0.011664653196930885, -0.28438444555316167, 0.5884795188903809, 0.16868948936462402): [(0.0, 0.029165378462401973), (0.4847647815546971, 0.8702880071145546)], (-1.4306638876544397, -2.5761057339110316, 0.02560884157289678, 0.02345816671337886, -0.6554859320269983, -0.5436061064349573): [(0.0, 0.36475392336316526), (0.38030458524579386, 0.46492933437321604), (0.49952181718266464, 0.7947649391050171), (0.8783861924511837, 1.0)]}

    for sample in samples:
        if tuple(sample) in Intervals.keys():
            print(f'skipping sample, len: {len(Intervals)}')
            continue
        # go to sampled configuration
        assistance_robot.move(sample)

        # compute intervals
        print(f"Computing intervals... current length: {len(Intervals.keys())}")
        Intervals[tuple(sample)] = compute_intervals()
        print(Intervals)

        assistance_robot.move_home()

    print(Intervals)


def reset_robots():
    task_robot.move_home()
    assistance_robot.move_home()


def execute_final():
    path = [[0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0],
            [0.804753839969635, -0.11332018793139653, 0.6386845747577112, -3.5985146961607875, -2.345687452946798,
             0.05820784717798233],
            [-0.41845876375307256, -0.9729586404613038, -0.004474210552871227, -0.9480648797801514, -1.4475892225848597,
             -0.4274819532977503],
            [0.494315505027771, -0.8066403430751343, -0.004446625709533691, -1.210972325210907, -1.896071736012594,
             0.5126177072525024],
            [-0.41845876375307256, -0.9729586404613038, -0.004474210552871227, -0.9480648797801514, -1.4475892225848597,
             -0.4274819532977503],
            [0.7405092120170593, -1.0121575158885499, 0.6384099165545862, -2.695364614526266, -2.3456791082965296,
             0.058171361684799194],
            [1.1083424091339111, -0.8869695228389283, -0.0053337812423706055, -1.175297812824585, -2.3454323450671595,
             0.9656500816345215],
            [0.804753839969635, -0.11332018793139653, 0.6386845747577112, -3.5985146961607875, -2.345687452946798,
             0.05820784717798233],
            [1.1083424091339111, -0.8869695228389283, -0.0053337812423706055, -1.175297812824585, -2.3454323450671595,
             0.9656500816345215]]
    timing_profile = [0.0, 4.6913742, 9.9922765, 16.88279395, 18.76299655, 22.65004799, 32.48562853, 57.69700707999999,
                      73.0]

    path = [[0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0],
            [0.494315505027771, -0.8066403430751343, -0.004446625709533691, -1.210972325210907, -1.896071736012594,
             0.5126177072525024],
            [-0.41845876375307256, -0.9729586404613038, -0.004474210552871227, -0.9480648797801514, -1.4475892225848597,
             -0.4274819532977503],
            [1.1083424091339111, -0.8869695228389283, -0.0053337812423706055, -1.175297812824585, -2.3454323450671595,
             0.9656500816345215],
            [0.494315505027771, -0.8066403430751343, -0.004446625709533691, -1.210972325210907, -1.896071736012594,
             0.5126177072525024],
            [-0.41845876375307256, -0.9729586404613038, -0.004474210552871227, -0.9480648797801514, -1.4475892225848597,
             -0.4274819532977503],
            [0.7405092120170593, -1.0121575158885499, 0.6384099165545862, -2.695364614526266, -2.3456791082965296,
             0.058171361684799194],
            [1.1083424091339111, -0.8869695228389283, -0.0053337812423706055, -1.175297812824585, -2.3454323450671595,
             0.9656500816345215],
            [0.804753839969635, -0.11332018793139653, 0.6386845747577112, -3.5985146961607875, -2.345687452946798,
             0.05820784717798233],
            [1.1083424091339111, -0.8869695228389283, -0.0053337812423706055, -1.175297812824585, -2.3454323450671595,
             0.9656500816345215],
            [0.7405092120170593, -1.0121575158885499, 0.6384099165545862, -2.695364614526266, -2.3456791082965296,
             0.058171361684799194]]
    timing_profile = [0.0, 3.7921456, 6.987630809999999, 10.04123541, 16.88279395, 18.76299655, 22.65004799,
                      32.48562853, 56.10284986999999, 67.70173008, 73.0]

    path = [[0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0], [-1.4306638876544397, -2.5761057339110316, 0.02560884157289678, 0.02345816671337886, -0.6554859320269983, -0.5436061064349573], [-1.7122295538531702, -2.518874307672018, -0.0005792640149593353, 0.45814983427014155, -0.13690025011171514, -1.0553653875934046], [-1.4306638876544397, -2.5761057339110316, 0.02560884157289678, 0.02345816671337886, -0.6554859320269983, -0.5436061064349573], [-1.7122295538531702, -2.518874307672018, -0.0005792640149593353, 0.45814983427014155, -0.13690025011171514, -1.0553653875934046], [-1.7499454657184046, -2.661220212975973, 0.003704849873678029, -0.4731615346721192, -0.5209363142596644, -0.0002592245685022476], [-1.4306638876544397, -2.5761057339110316, 0.02560884157289678, 0.02345816671337886, -0.6554859320269983, -0.5436061064349573], [-2.069029156361715, -3.0399538479247035, -0.011664653196930885, -0.28438444555316167, 0.5884795188903809, 0.16868948936462402], [-1.4306638876544397, -2.5761057339110316, 0.02560884157289678, 0.02345816671337886, -0.6554859320269983, -0.5436061064349573]]
    timing_profile = [0.0, 6.500455125, 7.537625625, 8.574796125, 9.611966624999999, 15.99990204, 18.980460705, 36.613563285, 43.5]

    reset_robots()
    print('Starting')
    sleep(2)
    task_robot.execute_task()
    assistance_robot.execute_path(path, timing_profile=timing_profile)
    exit(0)


def compute_task_time():
    avg_time = 0
    N = 10
    for _ in range(N):
        reset_robots()
        print('Starting')
        sleep(2)
        t = time()
        task_robot.execute_task()
        while task_robot.is_running():
            pass
        avg_time += (time() - t) / N
        print(f'Task time: {round(time() - t, 3)}')

    print(f'Average: {avg_time}')
    exit(0)


if __name__ == '__main__':
    # compute_task_time()
    execute_final()
    # exit(0)
    samples = [
        assistance_robot.home,
        [-1.9084609190570276, -3.096548696557516, -0.006988062057644129, -0.045498446827270556, -0.868300739918844,
         -0.00022489229311162262],
        [-1.7499454657184046, -2.661220212975973, 0.003704849873678029, -0.4731615346721192, -0.5209363142596644,
         -0.0002592245685022476],
        [-1.7122295538531702, -2.518874307672018, -0.0005792640149593353, 0.45814983427014155, -0.13690025011171514,
         -1.0553653875934046],
        [-1.8215530554400843, -2.5260197124876917, -0.00019462875206954777, 0.4559323030659179, -0.6241739431964319,
         -1.0540202299701136],
        [-2.069029156361715, -3.0399538479247035, -0.011664653196930885, -0.28438444555316167, 0.5884795188903809,
         0.16868948936462402],
        [-1.4306638876544397, -2.5761057339110316, 0.02560884157289678, 0.02345816671337886, -0.6554859320269983,
         -0.5436061064349573],
        [-1.6357839743243616, -2.802204748193258, 1.0883329550372522, -0.06645234048876958, -0.42808753648866826,
          -1.4271696249591272],
         [-1.8714473883258265, -3.053084989587301, 0.04420787492860967, -0.06963284433398442, -0.8167756239520472,
          0.035781990736722946]
    ]
    generate_from_list(samples)

'''
home
[0.4154955744743347, -3.0889584026732386, -0.004440600983798504, -0.08255477369342046, 1.1764490604400635, 0.009514331817626953]
[-0.011867348347799123, -2.579003473321432, -0.001514793373644352, 0.07376722871746821, 1.0491279363632202, 0.5725481510162354]
[-0.014177624379293263, -1.6043607197203578, 0.033509079610006154, 0.06766728937115474, -0.03832084337343389, -1.703691307698385]
[-1.1889117399798792, -0.901359037762024, -1.8020645380020142, -0.4623677295497437, 1.1054232120513916, -0.025052849446431935]
[-1.1465619246112269, -2.027121683160299, -1.9031667709350586, 0.6782716947742919, 1.0984952449798584, 0.06840498745441437]
[-1.2014954725848597, -2.2325173817076625, -1.1535048484802246, -0.04193039358172612, 1.1713902950286865, 0.12482447922229767]
[-1.2014835516559046, -2.2325054607787074, -1.1535197496414185, -0.0418885511210938, 1.1714805364608765, -2.3283541838275355]
[-1.228062931691305, -1.9083448849120082, -1.330953598022461, 0.0995076137730102, 1.2622976303100586, 0.0041425228118896484]
home


'''
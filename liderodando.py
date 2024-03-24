import time

from robo import robo
data_list = []


D = 0.2185
R = 0.0335
MIN_VELOCITY = 55
MAX_LINEAR_VELOCITY = 2.8*R
MAX_ANGULAR_VELOCITY = 2.8*R/D
velo_angular_roda = 6.33
v_linear = 0.106
v_angular = 0.4852
u_max = (v_linear + v_angular*D)/2

def find_velocity(points):

    print(points)

    p45, pm45, p15, pm15, p90, pm90, p30, pm30,p0 = points["45"], points["-45"], points["15"], points["-15"],\
        points["90"], points["-90"], points["30"], points["-30"],points["0"]

    v = 55
    ajuste_r = 1.2
    curva_suave = 0.7
    curva = 1

    if p0 > 0.30 and p15 > 0.30 and pm15 > 0.30:
        n_vl = v
        n_vr = v
        if p30 > 0.30 and pm30 > 30:
            if p30 < pm30:
                n_vl = v * curva_suave
                n_vr = v
            else:
                n_vl = v
                n_vr = v * curva_suave
    else:
        if (p90+p45) > (pm90+pm45):
            n_vl = v * curva
            n_vr = -v * curva
        else:
            n_vl = -v * curva
            n_vr = v * curva

    n_vr = n_vr * ajuste_r
    return n_vl, n_vr

def main():
    robs = robo()
    try:
        robs.set_velocity(0, 0, 0, 0)
        o_vl = 0
        o_vr = 0
        while True:
            robs.set_velocity(0,0,0,0)
            _, _, points = robs.interact()

            if points is None:
                continue
            print(points)

            n_vl, n_vr = find_velocity(points)
            robs.set_velocity(n_vl,n_vr, o_vl, o_vr)
            print(n_vl, n_vr)
            o_vl = n_vl
            o_vr = n_vr
            time.sleep(0.5)

    except KeyboardInterrupt:
        robs.disconect()

if __name__ == "__main__":
    main()

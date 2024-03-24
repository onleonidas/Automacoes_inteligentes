from robo import robo

data_list = []


D = 0.2185
R = 0.0335

def find_velocity( dl, dr, dc, dll, drr):
    vl, vr, v = 0, 0, 0
    wl, wr, w = 0, 0, 0
    sinal = 1
    MAX_LINEAR_VELOCITY = 75
    MAX_ANGULAR_VELOCITY = 25

    if abs(dll - drr) < 0.05:
        MAX_LINEAR_VELOCITY = 75
        MAX_ANGULAR_VELOCITY = 25
    elif abs(dll - drr) < 0.2:
        MAX_LINEAR_VELOCITY = 75 - 50*((0.2 - abs(dll - drr))/0.2)
        MAX_ANGULAR_VELOCITY = 25 + 50*((abs(dll - drr))/0.2 )
    else:
        MAX_LINEAR_VELOCITY = 25
        MAX_ANGULAR_VELOCITY = 75


    if dc < 0.15:
        vr, vl = 0, 0
    elif dc < 0.6:
        v = (dc - 0.2)*MAX_LINEAR_VELOCITY
        vr, vl = v, v
    else:
        vr, vl = MAX_LINEAR_VELOCITY,MAX_LINEAR_VELOCITY

    delta = dr - dl

    if abs(delta) > 0.5:
        if abs(delta) > 5:
            w = 0
        else:
            w = MAX_ANGULAR_VELOCITY

    elif abs(delta) < 0.05:
        w = 0

    else:
        w = abs(delta)*MAX_ANGULAR_VELOCITY

    if sinal == -1:
        wl = w
    else:
        wr = w

    ur = vr + wr
    ul = vl + wl

    if ur != 0:
        if ur < 50 and sinal == -1:
            if ur < 30:
                ur += 50
            ur += 30

    if ul != 0:
        if ul < 50 and sinal == 1:
            if ul < 30:
                ul += 50
            ul += 30

    if ur > 100:
        ur = 100
    if ul > 100:
        ul = 100


    return ul, ur

def main():
    robs = robo()

    try:
        o_vl = 100
        o_vr = 100
        robs.set_velocity(0, 0)
        while True:
            _, _ , points = robs.interact()
            print(robs.interact())
            dl, dr, dc, dll, drr = points["-40"], points["40"], points["0"], points["-80"], points["80"]

            n_vl, n_vr = find_velocity(dl,dr,dc, dll, drr)



            print(n_vl,n_vr)

            robs.set_velocity(n_vl,n_vr)

    except KeyboardInterrupt:
        robs.disconect()

if __name__ == "__main__":
    main()
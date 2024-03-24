from robo import robo
import time
def main():
    robs = robo()
    try:
        robs.set_velocity(0, 0, 0, 0)
        o_vl = 0
        o_vr = 0
        lista = []
        linha = {"ri":0, "li":0, "rv":0, "lv":0}
        with open('dados_robo1.csv', 'w') as f:
            #salva cabeçalho
            f.write("ri,li,rv,lv;\n")

        #range de 0 a 100 pulando de 5 em 5
        for i in range(0, 100, 5):
            for j in range(0, 100, 5):
                #seta a velocidade dos motores
                robs.set_velocity(i, j, o_vl, o_vr)
                o_vl = i
                o_vr = j
                rv, lv,_ = robs.interact()
                linha["ri"] = i * 6.28/100
                linha["li"] = j * 6.28/100
                linha["rv"] = rv
                linha["lv"] = lv
                lista.append(linha)
                time.sleep(2)

                #seta inverso da velocidade dos motores
                robs.set_velocity(-i, -j, o_vl, o_vr)
                o_vl = -i
                o_vr = -j
                rv, lv,_ = robs.interact()
                linha["ri"] = i
                linha["li"] = j
                linha["rv"] = rv
                linha["lv"] = lv
                lista.append(linha)
                time.sleep(2)

                #seta um positivo e um negativo
                robs.set_velocity(i, -j, o_vl, o_vr)
                o_vl = i
                o_vr = -j
                rv, lv,_ = robs.interact()
                linha["ri"] = i
                linha["li"] = j
                linha["rv"] = rv
                linha["lv"] = lv
                lista.append(linha)
                time.sleep(2)

                #seta um negativo e um positivo
                robs.set_velocity(-i, j, o_vl, o_vr)
                o_vl = -i
                o_vr = j
                rv, lv, _ = robs.interact()
                linha["ri"] = i
                linha["li"] = j
                linha["rv"] = rv
                linha["lv"] = lv
                lista.append(linha)
                time.sleep(2)

        #salva os dados em um arquivo csv
        with open('dados_robo1.csv', 'a') as f:
            #salva os dados como um linha do csv encerrando com ;\n
            for linha in lista:
                f.write(str(linha["ri"]) + "," + str(linha["li"]) + "," + str(linha["rv"]) + "," + str(linha["lv"]))
                f.write(";\n")
        lista.clear()
    except KeyboardInterrupt:
        robs.disconect()

if __name__ == "__main__":
    main()
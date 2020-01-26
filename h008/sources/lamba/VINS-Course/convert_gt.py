with open("/home/ubuntu/Desktop/euroc_mh05_groundtruth.csv") as f:
    with open("gt.txt", "wt") as fw:
        i=0
        for l in f:
            if i==0:i += 1;continue
            # timestamp, x, y,z, qw, qx, qy, qz
            l = l.strip().split(",")[:8]
            if len(l)==0:i += 1;continue
            l[0] = str(float(l[0]) / 1e9)        
            lw = " ".join(l)
            print(lw, file=fw)
            i += 1
        
        
        
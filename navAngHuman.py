math.degrees(pangles(ang)))

    # For the logic to work we need to have positive angles
    # But atan2() returns negatives as well
    def pangles(self,ang):
        if ang >= 0:
            return ang
        else:
            return (2 * math.pi + ang)


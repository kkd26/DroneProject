class Curve:
    def __init__(self):
        self.points = []
        self.valid = False


class PathGenerator:
    def __init__(self, model, ground_route):
        self.model = model
        self.ground_route = ground_route
        self.proposed_route = ground_route.copy()
        self.index = 0

    def __iter__(self):
        return self

    def __next__(self) -> Curve:
        try:
            self.index += 1
            return self.proposed_route[self.index - 1]
        except IndexError:
            raise StopIteration

    async def model_change_callback(self):
        """would be nice to have the info as to what changed, eg
          - which entity(s)?
          - which attribute(s)?
          - by how much?
        """
        pass

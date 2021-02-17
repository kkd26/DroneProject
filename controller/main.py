import generate_path
import asyncio


async def main():
    # connect to drone
    # connect to phone
    # init other modules and start their main loops
    generator = generate_path.PathGenerator(model=..., ground_route=...)


if __name__ == '__main__':
    asyncio.run(main())

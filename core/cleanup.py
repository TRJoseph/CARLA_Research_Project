import carla

def cleanup(client, actors):
    """Destroy actors using CARLA's batch command for efficiency"""
    actor_ids = []
    
    for actor in actors:
        if hasattr(actor, 'actor') and actor.actor is not None:
            actor_ids.append(actor.actor.id)
        elif hasattr(actor, 'id'):
            actor_ids.append(actor.id)
    
    if actor_ids:
        print(f"Destroying {len(actor_ids)} actors...")
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_ids])
        print("Cleanup complete!")
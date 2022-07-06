        def showImage(weak_self, frame, image):
            self = weak_self()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (int(image.height), int(image.width), 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = cv2.flip(array, 1)
            self.images[frame] = array.swapaxes(0, 1)


        def showImage_izquierda(weak_self, frame, image):
            self = weak_self()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (int(image.height), int(image.width), 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            #array = cv2.flip(array, 1)
            self.images[frame] = array.swapaxes(0, 1)
        

        self._camera_transforms = [
                (carla.Transform(carla.Location(x=bound_x/15, y=-0.4, z=1.15), carla.Rotation()), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=+0.6, y=-bound_y*0.59, z=1.03), carla.Rotation(yaw=180+15)), carla.AttachmentType.Rigid),
                #(carla.Transform(carla.Location(x=+0.6, y=-bound_y*1.59, z=1.03), carla.Rotation(yaw=-20)), carla.AttachmentType.Rigid),
                
                (carla.Transform(carla.Location(x=+0.6, y=bound_y*0.59, z=1.03), carla.Rotation(yaw=180-15)), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=-bound_x, z=1.1), carla.Rotation(yaw=180)), carla.AttachmentType.Rigid),

                (carla.Transform(carla.Location(x=bound_x/15, y=-0.4, z=1.15), carla.Rotation(yaw=270+30)), carla.AttachmentType.Rigid), ##izquierda
               ]
                             
        weak_self = weakref.ref(self)

        sensor_left_bp = bp_library.find('sensor.camera.rgb')
        sensor_left_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_left_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.2)))
        sensor_left_bp.set_attribute('fov', str(120))

        sensor_r_bp = bp_library.find('sensor.camera.rgb')
        sensor_r_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_r_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.2)))
        sensor_r_bp.set_attribute('fov', str(30))


        sensor_back_bp = bp_library.find('sensor.camera.rgb')
        sensor_back_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_back_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.1)))
        sensor_back_bp.set_attribute('fov', str(90))

        
        #new code

        sensor_izquierda_bp = bp_library.find('sensor.camera.rgb')
        sensor_izquierda_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_izquierda_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.2)))
        sensor_izquierda_bp.set_attribute('fov', str(80))


        ## end new code


        self.sensor_back = parent_actor.get_world().spawn_actor(
                sensor_back_bp,
                self._camera_transforms[3][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_back.listen(lambda image: showImage(weak_self, "back",image))

        self.sensor_left = parent_actor.get_world().spawn_actor(
                sensor_left_bp,
                self._camera_transforms[1][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_left.listen(lambda image: showImage(weak_self, "left",image))

        self.sensor_right = parent_actor.get_world().spawn_actor(
                sensor_r_bp,
                self._camera_transforms[2][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_right.listen(lambda image: showImage(weak_self, "right",image))


        #new code

        self.sensor_izquierda = parent_actor.get_world().spawn_actor(
                sensor_izquierda_bp,
                self._camera_transforms[4][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_izquierda.listen(lambda image: showImage_izquierda(weak_self, "izquierda",image))


        #end new code